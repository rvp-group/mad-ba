# Requirements
- Docker

# Install

# Run
### Create Docker Image and Container
- build image: ```./build_image.sh```
- adjust shared path and start container: ```./run_container.sh```
### Clone repositories (inside container)
```bash
# SRRG
cd /catkin_ws/src && mkdir srrg && cd srrg && \
git clone https://kcwian@bitbucket.org/kcwian/srrg2_core.git -b devel && \
git clone https://kcwian@bitbucket.org/kcwian/srrg2_executor.git -b devel && \
git clone https://kcwian@bitbucket.org/kcwian/srrg2_solver.git -b devel && \
git clone https://kcwian@bitbucket.org/kcwian/srrg_cmake_modules.git -b master && \
git clone https://kcwian@bitbucket.org/kcwian/srrg2_qgl_viewport.git -b devel && \
# LOAM
cd /catkin_ws/src && mkdir LOAM && cd LOAM && \
git clone https://kcwian@bitbucket.org/kcwian/loam-opensource.git -b master && \
git clone https://kcwian@bitbucket.org/put_adas_ub/rosbag_player.git -b master && \
git clone https://kcwian@bitbucket.org/put_adas_ub/localize_main.git -b master  && \
# Plugins
cd /catkin_ws/src && mkdir plugins && cd plugins && \
git clone https://kcwian@bitbucket.org/kcwian/rviz_visual_tools.git -b noetic-devel && \
git clone https://kcwian@bitbucket.org/kcwian/surfel_cloud_rviz_plugin.git -b master && \
# Main repo
cd /catkin_ws/src && \
git clone https://kcwian@bitbucket.org/kcwian/structure_refinement.git -b rawOptimization
```

### Build the code
```bash
cd /catkin_ws
catkin build -DCMAKE_BUILD_TYPE=Release
```

### Download dataset (without .bag)
```bash
# Dataset
cd /root/share && \
git clone https://kcwian@bitbucket.org/kcwian/dataset.git
```