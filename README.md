<div align="center">
    <h1>MAD-BA</h1>
    <h3>3D LiDAR Bundle Adjustment &mdash; from Uncertainty Modelling to Structure Optimization</h3>
</div>


<div align="center">
  <!-- Video Thumbnail -->
  <a href="https://www.youtube.com/watch?v=Uz1ST_lP8r4" target="_blank" style="display: inline-block;">
    <img src="https://img.youtube.com/vi/Uz1ST_lP8r4/0.jpg" style="width: 60%; display: block;">
  </a>
  <br>
  <!-- Play Button -->
  <a href="https://www.youtube.com/watch?v=Uz1ST_lP8r4" target="_blank" style="display: inline-block;">
    <img src="https://upload.wikimedia.org/wikipedia/commons/b/b8/YouTube_play_button_icon_%282013%E2%80%932017%29.svg" 
         style="width: 50px; height: auto; margin-left: 5px;">
  </a>
</div>

## Build and Run
The MAD-BA was developed and tested on Ubuntu 20.04 with ROS Noetic.
#### Create Docker Image and Container
- Clone repository
```bash
git clone https://github.com/rvp-group/mad-ba.git
```
- Build docker image and create container: 
```bash
cd mad_ba && ./build_and_run.sh
```

#### Build MAD-BA
- Start and enter container: 
```bash
docker start mad_ba && docker exec -it mad_ba bash
```
- Build ROS packages (inside container): 
```bash
cd /catkin_ws && catkin build -DCMAKE_BUILD_TYPE=Release
```

#### Prepare data
- Download example `.bag` file (inside container): 
```bash
cd /root/share/dataset/NewerCollege/quad_easy/bag && bash download_quad.sh # quad_easy sequence
cd /root/share/dataset/VBR/Spagna/bag && bash download_spagna.sh # Spagna sequence
```
- Preprocess `.bag` file by reordering message using their timestamps and add inital trajectory as `nav_msgs/Odometry` messages:
```bash
cd /root/share/dataset/NewerCollege/quad_easy && python3 process_bag.py  # quad_easy sequence
cd /root/share/dataset/VBR/Spagna && python3 process_bag.py  # Spagna sequence
```

#### Run MAD-BA
- Start roscore in background: 
```bash
screen -dmS roscore roscore
```
- Run MAD-BA: 
```bash
roscd mad_ba && rosrun mad_ba main_app -c config/NewerCollege/quad_easy_fast.config # quad_easy sequence
roscd mad_ba && rosrun mad_ba main_app -c config/VBR/spagna.config # Spagna sequence
```

#### Output
- Check **./docker_shared/output/** folder:
  - `tum` - contains the optimized trajectory for each iteration
  - `pcd` - contains the optimized map for each iteration

#### Parameters
To modify parameters edit `.config` file for given sequence, located in `/catkin_ws/src/mad_ba/config` in Docker container:

```json
"filename":           # path to .bag file
"topics":             # topics for point cloud and odometry messages
"clouds_to_process":  # number of point clouds in .bag file
"decimate_real_data": # decimation - increase for longer sequences
"iter_num":           # number of BA iterations
```
