FROM osrf/ros:noetic-desktop-full

# SETUP ENVS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND noninteractive

# INSTALL SOME ESSENTIAL PROGRAMS
RUN apt update && apt install -y git wget bash-completion unzip python3-pip ros-$ROS_DISTRO-vision-opencv ros-$ROS_DISTRO-message-filters \
        libopencv-dev python3-opencv python3-dev python3-numpy python3-yaml libeigen3-dev libsuitesparse-dev libboost-dev \
        libboost-iostreams-dev libboost-filesystem-dev libboost-date-time-dev libboost-serialization-dev libboost-thread-dev \
        libblas-dev liblapack-dev libgflags-dev libgoogle-glog-dev libqglviewer-dev-qt5 vim openssh-server python3-catkin-tools \
        ros-$ROS_DISTRO-grid-map ros-$ROS_DISTRO-eigen-stl-containers ros-$ROS_DISTRO-graph-msgs screen nano \
        ros-$ROS_DISTRO-ublox-msgs gnuplot && \
        pip install gdown evo packaging termplotlib && \
        mkdir -p /catkin_ws/src/ && \
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
        echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

COPY ./modules/ /catkin_ws/src/

WORKDIR /catkin_ws/src
ENTRYPOINT bash
