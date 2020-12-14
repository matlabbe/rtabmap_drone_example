# mavros_nav_2d
Example of using move_base with mavros/px4 and rtabmap visual SLAM

## Dependencies

```bash
sudo apt install ros-melodic-rtabmap-ros ros-melodic-gazebo-dev

# Firmware/px4 (use version v1.8.2!)
cd ~
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.8.2
sudo pip install numpy toml
make posix_sitl_default gazebo
# (do ctrl-c in terminal to close gazebo)
echo "source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/posix_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware:~/Firmware/Tools/sitl_gazebo" >> ~/.bashrc
source ~/.bashrc

# To work with mavros 0.29.2, we need mavlink 3.3.1 under melodic
cd ~
wget https://github.com/mavlink/mavlink-gbp-release/archive/release/melodic/mavlink/2020.3.3-1.zip
unzip 2020.3.3-1.zip
cd mavlink-gbp-release-release-melodic-mavlink-2020.3.3-1
mkdir build
cd build
sudo apt install libxslt1-dev
pip install -U future lxml
cmake ..
make -j4
sudo make install

cd ~/catkin_ws/src
# To work with PX4/Firmware 1.8.2, we need mavros 0.29.2
git clone https://github.com/mavlink/mavros.git && cd mavros && git checkout 0.29.2 && cd ..
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
