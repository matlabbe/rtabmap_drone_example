/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	enum ControlState {
		kPositionControl,
		kVelocityControl
	};
public:
	OffboardControl() :
		Node("offboard_control"),
		use_sim_time_(false)
	{
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// Subscription to listen for Twist commands
		twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OffboardControl::twist_callback, this, std::placeholders::_1));
		
		// Subscription to get the current vehicle position
        local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", rclcpp::QoS(10).best_effort(), std::bind(&OffboardControl::local_pos_callback, this, std::placeholders::_1));
	
		status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(), std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
			
		ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", rclcpp::QoS(10).best_effort(), std::bind(&OffboardControl::vehicle_cmd_ack_callback, this, std::placeholders::_1));

		joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&OffboardControl::joy_callback, this, std::placeholders::_1));


    	this->get_parameter("use_sim_time", use_sim_time_);

		offboard_setpoint_counter_ = 0;

		current_goal_.x = 0;       // North position in NED earth-fixed frame, (metres)
		current_goal_.y = 0;       // East position in NED earth-fixed frame, (metres)
		current_goal_.z = -1.3;    // Down position (negative altitude) in NED earth-fixed frame, (metres)
		current_goal_.heading = 0; // Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
		
		control_State_ = kPositionControl;
		velocity2d_ = true;
		last_request_ = this->get_clock()->now();
		arming_stamp_ = this->get_clock()->now();

		auto timer_callback = [this]() -> void {

			if(offboard_setpoint_counter_ == 10) {
				RCLCPP_INFO(get_logger(), "Setting offboard mode...");
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}
			else if(offboard_setpoint_counter_ == 20) {
				if(!vehicle_status_.pre_flight_checks_pass ||
					vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
					offboard_setpoint_counter_ = 0;
					RCLCPP_WARN(get_logger(), 
						"Offboard could not be set, make sure the vehicle position "
						"is published or check if there are some preflight checks failing... "
						"will try again in 1 second.");
				}
				else {
					this->arm();
				}
			}
			
			if (offboard_setpoint_counter_ >= 21)
			{
				update_state();
			}
			else
			{
				++offboard_setpoint_counter_;
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:

	void update_state() {
		if((this->get_clock()->now() - last_request_ > rclcpp::Duration::from_seconds(1.0)))
		{
			if(!vehicle_status_.pre_flight_checks_pass || vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
			{
				RCLCPP_ERROR(get_logger(), "Pre-flight checks are failing and offboard is not enabled, shutting down!");
				rclcpp::shutdown();
			}
			else if(get_clock()->now().seconds() - twist_stamp_.seconds() < 1) // valid joystick command
			{
				if(vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
					(twist_.linear.z < -0.4 && twist_.angular.z < -0.4))// left joystick down-right
				{
					this->arm();
				}
				else if(
					vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
					twist_.linear.z < -0.4 && twist_.angular.z > 0.4) // left joystick down-left
				{
					this->disarm();
				}
			}
			last_request_ = get_clock()->now();
		}
	}

	void arm()
	{
		arming_stamp_ = get_clock()->now();
		current_goal_ = local_pose_;
		current_goal_.z = current_goal_.z - 1.3; // take-off 1.3 meter over current position 
		RCLCPP_INFO(get_logger(), "Vehicle arming..");
		RCLCPP_INFO(get_logger(), "Take off at 1.3 meter... to position=(%f,%f,%f) heading=%f",
				current_goal_.x,
				current_goal_.y,
				current_goal_.z,
				current_goal_.heading);
		control_State_ = kPositionControl;
		publish_vehicle_command(
			VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			VehicleCommand::ARMING_ACTION_ARM);
	}

	void disarm()
	{
		RCLCPP_INFO(get_logger(), "Vehicle disarming...");
		publish_vehicle_command(
			VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			VehicleCommand::ARMING_ACTION_DISARM);
	}

	void publish_offboard_control_mode()
	{
		OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = true;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint()
	{
		if(control_State_ != kPositionControl &&
		   rclcpp::Time(local_pose_.timestamp*1000).seconds() - twist_stamp_.seconds() > .1)
		{
			//switch to position mode with last position
			control_State_ = kPositionControl;
			current_goal_ = local_pose_;
			RCLCPP_INFO(get_logger(), "Switch to position control (x=%f, y=%f, z=%f, yaw=%f)",
					current_goal_.x, current_goal_.y, current_goal_.z, current_goal_.heading);
		}

		px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->now().nanoseconds() / 1000;
        
		double yaw = local_pose_.heading;
		double cosyaw = cos(yaw);
		double sinyaw = sin(yaw);

        // Use twist message for XY velocity control
        msg.velocity[0] = control_State_ == kVelocityControl? twist_.linear.x * cosyaw + (twist_.linear.y)*sinyaw : NAN;
        msg.velocity[1]= control_State_ == kVelocityControl ?  twist_.linear.x * sinyaw + (-twist_.linear.y)*cosyaw : NAN;
		msg.velocity[2]= control_State_ == kVelocityControl && !velocity2d_ ?  -twist_.linear.z : NAN;
		msg.yawspeed = control_State_ == kVelocityControl ? -twist_.angular.z : NAN;

		msg.position[0] = control_State_ == kPositionControl ? current_goal_.x : NAN;
		msg.position[1] = control_State_ == kPositionControl ? current_goal_.y : NAN;
        msg.position[2] = control_State_ == kPositionControl ? current_goal_.z : velocity2d_ ? current_goal_.z : NAN;
		msg.yaw = control_State_ == kPositionControl ? current_goal_.heading : NAN; // [-PI:PI]

		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publisher_->publish(msg);
	}

	void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
        twist_ = *msg;
		twist_stamp_ = this->get_clock()->now();

		if(twist_stamp_.seconds() - arming_stamp_.seconds() > 5.0 && control_State_ == kPositionControl)
		{
			RCLCPP_INFO(get_logger(), "Switch to velocity control");
			control_State_ = kVelocityControl;
		}
    }

    void local_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
	{
        local_pose_ = *msg;
		if(use_sim_time_) {
			local_pose_.timestamp = this->get_clock()->now().nanoseconds()/1000;
		}
    }

	void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
	{
        vehicle_status_ = *msg;
    }

	void vehicle_cmd_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
	{
      	if(msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
			RCLCPP_INFO(get_logger(), "Command accepted!");
		}
		else {
			RCLCPP_ERROR(get_logger(), "Command rejected!");
		}
    }

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
		if(msg->buttons[5] == 1)
		{
			// When holding right trigger, accept velocity in Z
			velocity2d_ = false;
		}
		else
		{
			velocity2d_ = true;
		}
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	double target_altitude_ = -1.3;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_subscriber_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscriber_;
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	geometry_msgs::msg::Twist twist_;
	rclcpp::Time twist_stamp_;
	rclcpp::Time arming_stamp_;

	VehicleLocalPosition local_pose_;
	VehicleLocalPosition current_goal_;
	px4_msgs::msg::VehicleStatus vehicle_status_;

	ControlState control_State_;
	bool velocity2d_;

	bool use_sim_time_;

	rclcpp::Time last_request_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}