# mavros_nav_2d
Example of using move_base with mavros/px4 and rtabmap visual SLAM

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
