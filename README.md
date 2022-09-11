# rtabmap_drone_example
2D navigation example of a drone using [move_base](http://wiki.ros.org/move_base) with [mavros](http://wiki.ros.org/mavros)/[px4](https://github.com/PX4/PX4-Autopilot) and [rtabmap](wiki.ros.org/rtabmap_ros) visual SLAM. 

Overview video ([click](https://youtu.be/A487ybS7E4E) to watch on Youtube):

[![Youtube](https://i.imgur.com/UKLtD7L.gif)](https://youtu.be/A487ybS7E4E)

## Install

### Docker (recommended)
To make it simple, create the following docker image (nvidia GPU required):
```bash
git clone https://github.com/matlabbe/rtabmap_drone_example.git
cd rtabmap_drone_example
docker build -t rtabmap_drone_example -f docker/Dockerfile .
```

### Host
Follow instructions from [docker/Dockerfile](https://github.com/matlabbe/rtabmap_drone_example/blob/master/docker/Dockerfile) to install dependencies. 

## Usage

### Docker (recommended)

Launch the simulator:
```bash
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
  --privileged \
  --network=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --runtime=nvidia \
  rtabmap_drone_example \
  roslaunch rtabmap_drone_example gazebo.launch
```

Launch VSLAM:
```bash
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
  --privileged \
  --network=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --runtime=nvidia \
  rtabmap_drone_example \
  roslaunch rtabmap_drone_example slam.launch
```

Launch rviz:
```bash
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
  --privileged \
  --network=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --runtime=nvidia \
  rtabmap_drone_example \
  roslaunch rtabmap_drone_example rviz.launch
```

Arm and take off:
```bash
docker run -it --rm \
  --privileged \
  --network=host \
  rtabmap_drone_example \
  rosrun rtabmap_drone_example offboard
```

### Host

```bash
roslaunch rtabmap_drone_example gazebo.launch
roslaunch rtabmap_drone_example slam.launch
roslaunch rtabmap_drone_example rviz.launch

rosrun rtabmap_drone_example offboard
```


## Control
 * Autonomous control: use "2D Nav Goal" button in RVIZ to set a goal to reach
 * Manual control: If a joystick is plugged, you can send twists by holding L1 and moving the joysticks. Hold L1+L2 with left joystick down to land (be gentle to land smoothly), then hold left joystick in bottom-right position to disarm after the drone is on the ground.
 

![](https://raw.githubusercontent.com/matlabbe/rtabmap_drone_example/master/doc/example.jpg)

