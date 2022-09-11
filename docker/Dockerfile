# cd rtabmap_drone_example
# docker build -t rtabmap_drone_example -f docker/Dockerfile .

FROM ros:noetic-perception

RUN apt update && apt install \
    ros-noetic-gazebo-dev \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-geographic-msgs \
    ros-noetic-dwa-local-planner \
    ros-noetic-rtabmap-ros \
    ros-noetic-mavlink \
    ros-noetic-gazebo-ros \
    ros-noetic-control-toolbox \
    ros-noetic-move-base \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    libgeographic-dev \
    geographiclib-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    python3-pip \
    git -y

RUN pip3 install numpy toml packaging jinja2 empy pyros-genmsg

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /root/

RUN git clone https://github.com/PX4/PX4-Autopilot.git &&\
    cd PX4-Autopilot && \
    git checkout v1.12.3 && \
    git submodule update --init --recursive && \
    make -j8 px4_sitl_default gazebo && \
    make px4_sitl_default install && \
    cd && \
    rm -rf PX4-Autopilot

# To work with PX4/Firmware 1.12.3, mavros 1.8.0 or 1.9.0 releases should be used
# (With mavros master branch there are a lot of "Detected jump back in time" TF errors)
RUN source /ros_entrypoint.sh && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    catkin_init_workspace && \
    git clone https://github.com/mavlink/mavros.git && \
    git clone https://github.com/SyrianSpock/realsense_gazebo_plugin.git && \
    echo "install(DIRECTORY models DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION})" >> realsense_gazebo_plugin/CMakeLists.txt && \
    cd mavros && \
    git checkout 1.9.0 && \
    ./mavros/scripts/install_geographiclib_datasets.sh && \
    cd ~/catkin_ws && \
    catkin_make -j8 -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install && \
    cd && \
    rm -rf ~/catkin_ws/devel ~/catkin_ws/build ~/catkin_ws/src/mavros ~/catkin_ws/src/realsense_gazebo_plugin

COPY . /root/catkin_ws/src/rtabmap_drone_example

RUN source /ros_entrypoint.sh && \
    cd ~/catkin_ws && \
    catkin_make -j8 -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install && \
    cd && \
    rm -rf ~/catkin_ws
    
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "#!/bin/bash" >> /entrypoint.sh && \
    echo "set -e" >> /entrypoint.sh && \
    echo "source /opt/ros/noetic/setup.bash" >> /entrypoint.sh && \
    echo "source /usr/local/px4/Tools/setup_gazebo.bash /usr/local/px4 /usr/local/px4/build/px4_sitl_default" >> /entrypoint.sh && \
    echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/usr/local/px4:/usr/local/px4/Tools/sitl_gazebo" >> /entrypoint.sh && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/rtabmap_drone_example/models" >> /entrypoint.sh && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/realsense_gazebo_plugin/models" >> /entrypoint.sh && \
    echo "exec \"\$@\"" >> /entrypoint.sh && \
    chmod +x /entrypoint.sh
    
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
