FROM giorgiogrisetti/ubuntu20.04_cuda_ros_noetic:latest
ENV DEBIAN_FRONTEND=noninteractive
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1

MAINTAINER Giorgio Grisetti <grisetti@diag.uniroma1.it>

# # create workspace
RUN mkdir -p /workspace/src

# # clone required pkgs
RUN cd /workspace/src && git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git   
RUN echo "getting core"
RUN cd /workspace/src && git clone https://gitlab.com/srrg-software/srrg2_core.git && cd srrg2_core && git checkout devel 
RUN cd /workspace/src && git clone https://gitlab.com/srrg-software/srrg2_qgl_viewport.git && cd srrg2_qgl_viewport && git checkout devel
RUN cd /workspace/src && git clone https://gitlab.com/srrg-software/srrg2_executor.git && cd srrg2_executor && git checkout devel
RUN cd /workspace/src && git clone https://gitlab.com/srrg-software/srrg2_config_visualizer && cd srrg2_config_visualizer && git checkout devel


# build everything
RUN . /opt/ros/noetic/setup.sh && cd /workspace/ && catkin init && catkin build


#WORKDIR /
CMD ["bash"]

RUN echo "alias ws2='source /workspace/devel/setup.bash'" >> ~/.bashrc
RUN echo "alias confviz='/workspace/devel/lib/srrg2_config_visualizer/app_node_editor" >> ~/.bashrc

# tuning stuff
# updating global variable path
ENV PATH="$PATH:/workspace/devel/lib/srrg2_executor_nogui"

