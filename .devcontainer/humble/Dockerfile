
FROM ros:humble-perception-jammy

# ros dev tools
RUN apt update && \
    apt install -y ros-dev-tools python3-pip 
   
RUN pip install -U empy==3.3.4 pyros-genmsg kconfiglib jsonschema jinja2

# XRCE-DDS Agent
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j6 && \
    make install && \
    ldconfig /usr/local/lib/ && \
    cd ../../ && \
    rm -rf Micro-XRCE-DDS-Agent

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -ex && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -a -G sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG video ${USERNAME}

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

USER ${USERNAME}
ENV USER=${USERNAME}
WORKDIR /home/${USERNAME}

RUN mkdir -p ros2_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
RUN echo "source /usr/share/bash-completion/completions/git" >> .bashrc

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

RUN sudo apt update && \
    sudo apt install -y \
    ros-humble-ros-gzharmonic \
    ros-humble-imu-filter-madgwick \
    ros-humble-rtabmap-ros \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-rqt-reconfigure

RUN source /opt/ros/humble/setup.bash && \
    cd ros2_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    # Add rtabmap_costmap_plugins (not yet released as binaries)
    git clone https://github.com/introlab/rtabmap_ros && \
    mv rtabmap_ros/rtabmap_costmap_plugins . && \
    rm -rf rtabmap_ros && \
    cd .. && \
    colcon build && \
    rm -rf build src/*

RUN echo "source ~/ros2_ws/install/setup.bash" >> .bashrc && \
    echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> .bashrc && \
    echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1" >> .bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> .bashrc

# copy our world in PX4 repo
COPY --chown=${USER_GID}:${USER_GID} models/apt ./PX4-Autopilot/Tools/simulation/gz/models/apt
COPY --chown=${USER_GID}:${USER_GID} worlds/apt.sdf ./PX4-Autopilot/Tools/simulation/gz/worlds/apt.sdf

# copy modified OakD-Lite model
COPY --chown=${USER_GID}:${USER_GID} models/OakD-Lite ./PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite

# copy our modified 4002_gz_x500_depth px4 config to enable vision pose (disable GPS)
COPY --chown=${USER_GID}:${USER_GID} config/4002_gz_x500_depth ./PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4002_gz_x500_depth

# build PX4
RUN cd ./PX4-Autopilot && \
    make -j6 px4_sitl