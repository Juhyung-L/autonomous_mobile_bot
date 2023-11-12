FROM osrf/ros:humble-desktop-jammy

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

##### Setup Gazebo #####
# essential install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    binutils \
    mesa-utils \
    kmod \
    x-window-system \
    curl \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN curl -sSL http://get.gazebosim.org | sh

##### Install dependencies #####
RUN git clone https://github.com/Juhyung-L/autonomous_mobile_bot.git \
    && source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install -y -r --from-paths autonomous_mobile_bot --ignore-src --rosdistro=humble -y \
    && rm -rf autonomous_mobile_bot

RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git \
    && source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install -y -r --from-paths rplidar_ros --ignore-src --rosdistro=humble -y \
    && rm -rf rplidar_ros

WORKDIR /home/dev_ws

# setup bashrc settings
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc && \
    echo "alias b='source /home/dev_ws/install/setup.bash'" >> ~/.bashrc

# setup entrypoint
COPY ./gazebo_entrypoint.sh /
RUN chmod +x /gazebo_entrypoint.sh

ENTRYPOINT ["/gazebo_entrypoint.sh"]

# personal packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
   vim \
   gdb \
   gdbserver \
   libserial-dev \
   && rm -rf /var/lib/apt/lists/*
