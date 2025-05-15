FROM giorgiogrisetti/ubuntu_20.04_cuda_ros_noetic_srrg2_core:latest
ENV DEBIAN_FRONTEND=noninteractive
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

MAINTAINER Giorgio Grisetti <grisetti@diag.uniroma1.it>

RUN cd /workspace/src && git clone https://gitlab.com/srrg-software/srrg2_solver && cd srrg2_solver && git checkout devel

# build everything
RUN . /opt/ros/noetic/setup.sh && cd /workspace/ && catkin init && catkin build


#WORKDIR /
CMD ["bash"]

