## Create Docker Image and Container
- Build image: ```./build_image.sh```
- Create ontainer: ```./run_container.sh```

## Build MAD-BA
- Start and enter container: ```docker start mad_ba && docker exec -it mad_ba bash```
- Build ROS packages: ```cd /catkin_ws && catkin build -DCMAKE_BUILD_TYPE=Release```

## Prepare data
- Download .bag file ie.: ```cd /root/share/dataset/VBR/Spagna/bag && bash download_spagna.sh```
- Preprocess .bag file: ```cd /root/share/dataset/VBR/Spagna && python3 process_bag.py```

## Run MAD-BA
- Run MAD-BA: ```roscd mad_ba && 


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