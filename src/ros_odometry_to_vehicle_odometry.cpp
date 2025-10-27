

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mutex>
#include <chrono>
using namespace std::chrono_literals;

class RosOdometry2VehicleOdometry : public rclcpp::Node
{
public:
	RosOdometry2VehicleOdometry() : 
		Node("ros_odometry_to_vehicle_odometry"),
		map_frame_id_("map"),
		repeat_odom_(false)
	{
		map_frame_id_ = this->declare_parameter("map_frame_id", map_frame_id_);
		repeat_odom_ = this->declare_parameter("repeat_odom", repeat_odom_);

        vehicle_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RosOdometry2VehicleOdometry::odom_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	
		if(repeat_odom_) {
			// Publish pose at steady rate and restamp with now
			auto timer_callback = [this]() -> void {
				px4_msgs::msg::VehicleOdometry msg;
				{
					std::scoped_lock lock(mutex_); 
					msg = current_vehicle_odometry_;
					msg.timestamp = this->get_clock()->now().nanoseconds();
				}
				
				vehicle_odometry_publisher_->publish(msg);
			};
			timer_ = this->create_wall_timer(50ms, timer_callback);
		}
	}

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
	{
		try {
			geometry_msgs::msg::TransformStamped map_to_odom = tf_buffer_->lookupTransform(
				map_frame_id_, 
				odometry->header.frame_id, 
				rclcpp::Time(odometry->header.stamp), 
				rclcpp::Duration::from_seconds(0.01));

			px4_msgs::msg::VehicleOdometry msg;
			msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
			msg.timestamp = rclcpp::Time(odometry->header.stamp).nanoseconds();

			// Convert odometry pose in map frame
			Eigen::Quaterniond orientationENU;
			tf2::fromMsg(odometry->pose.pose.orientation, orientationENU);
			tf2::doTransform(orientationENU, orientationENU, map_to_odom);

			Eigen::Vector3d positionENU;
			tf2::fromMsg(odometry->pose.pose.position, positionENU);
			tf2::doTransform(positionENU, positionENU, map_to_odom);

			Eigen::Quaterniond map_to_odom_quat;
			tf2::fromMsg(map_to_odom.transform.rotation, map_to_odom_quat);
			px4_ros_com::frame_transforms::Covariance6d covENU = px4_ros_com::frame_transforms::transform_frame(odometry->pose.covariance, map_to_odom_quat);
			
			Eigen::Quaterniond orientationNED = px4_ros_com::frame_transforms::ros_to_px4_orientation(orientationENU);
			Eigen::Vector3d positionNED = px4_ros_com::frame_transforms::transform_static_frame(positionENU, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
			px4_ros_com::frame_transforms::Covariance6d covNED = px4_ros_com::frame_transforms::transform_static_frame(covENU, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

			msg.q[0] = orientationNED.w();
			msg.q[1] = orientationNED.x();
			msg.q[2] = orientationNED.y();
			msg.q[3] = orientationNED.z();
			msg.position[0] = positionNED.x();
			msg.position[1] = positionNED.y();
			msg.position[2] = positionNED.z();
			msg.position_variance[0] = covNED[0];
			msg.position_variance[1] = covNED[7];
			msg.position_variance[2] = covNED[14];
			msg.orientation_variance[0] = covNED[21];
			msg.orientation_variance[1] = covNED[28];
			msg.orientation_variance[2] = covNED[35];

			// Twist is published in base frame, just convert to NED
			msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
			Eigen::Vector3d linearROS, angularROS;
			tf2::fromMsg(odometry->twist.twist.linear, linearROS);
			tf2::fromMsg(odometry->twist.twist.angular, angularROS);
			Eigen::Vector3d linearFRD = px4_ros_com::frame_transforms::transform_static_frame(linearROS, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
			msg.velocity[0] = linearFRD.x();
			msg.velocity[1] = linearFRD.y();
			msg.velocity[2] = linearFRD.z();
			Eigen::Vector3d angularFRD = px4_ros_com::frame_transforms::transform_static_frame(angularROS, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
			msg.angular_velocity[0] = angularFRD.x();
			msg.angular_velocity[1] = angularFRD.y();
			msg.angular_velocity[2] = angularFRD.z();
			px4_ros_com::frame_transforms::Covariance6d velocityCovNED = px4_ros_com::frame_transforms::transform_static_frame(odometry->twist.covariance, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
			msg.velocity_variance[0] = velocityCovNED[0];
			msg.velocity_variance[1] = velocityCovNED[7];
			msg.velocity_variance[2] = velocityCovNED[14];

			if(repeat_odom_)
			{
				std::scoped_lock lock(mutex_); 
				current_vehicle_odometry_ = msg;
			}
			else {
				vehicle_odometry_publisher_->publish(msg);
			}
		}
		catch(tf2::LookupException & e)
		{
			RCLCPP_ERROR(this->get_logger(), e.what());
		}
		catch(tf2::ExtrapolationException & e)
		{
			RCLCPP_ERROR(this->get_logger(), e.what());
		}
	}

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	rclcpp::TimerBase::SharedPtr timer_;
	px4_msgs::msg::VehicleOdometry current_vehicle_odometry_;
	std::mutex mutex_;

	std::string map_frame_id_;
	bool repeat_odom_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RosOdometry2VehicleOdometry>());
	rclcpp::shutdown();
	return 0;
}