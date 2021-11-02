# mavros_nav_2d
Example of using move_base with mavros/px4 and rtabmap visual SLAM

## Dependencies

This following example has been tested on ROS Noetic. For ROS Melodic, checkout this tag: https://github.com/matlabbe/mavros_nav_2d/tree/px4_v1-8-2_mavros_v0-29-2

```bash
sudo apt install ros-noetic-rtabmap-ros ros-noetic-gazebo-dev

# PX4 (tested on version v1.11.3)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.11.3
git submodule update --init --recursive
sudo pip3 install numpy toml packaging jinja2 empy numpy
make px4_sitl_default gazebo
# (do ctrl-c in terminal to close gazebo)
echo "source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot:~/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc
source ~/.bashrc

cd ~/catkin_ws/src
# To work with PX4/Firmware 1.11.3, mavros 1.8.0 or 1.9.0 releases should be used
# (With mavros master branch there are a lot of "Detected jump back in time" TF errors)
git clone https://github.com/mavlink/mavros.git && cd mavros && git checkout 1.9.0 && cd ..
git clone https://github.com/SyrianSpock/realsense_gazebo_plugin.git

cd ~/catkin_ws
catkin_make
```

## Usage

```
roslaunch mavros_nav_2d gazebo.launch
roslaunch mavros_nav_2d slam.launch
roslaunch mavros_nav_2d rviz.launch

# Arm and take off:
rosrun mavros_nav_2d offboard

# Manual control: If a joystick is plugged, you can send twist by holding L1 and moving the joysticks
# Autonomous control: use "2D Nav Goal" button in RVIZ to set a goal to reach 

```
![](https://raw.githubusercontent.com/matlabbe/mavros_nav_2d/master/doc/example.jpg)
