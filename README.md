## Download NewerCollege Dataset
- Download *quad-easy* sequence: ```gdown https://drive.google.com/uc?id=1hF2h83E1THbFAvs7wpR6ORmrscIHxKMo```

## Prepare .bag file
- Download .bag sequence
- Process it with any LiDAR odometry software to have initial trajectory
- Process the .bag file (you can use `/scripts/process_bag.py` as an example): 
    - Reorder the .bag file using timestamp of each message
    - Add inital trajectory as `nav_msgs/Odometry` (poses should be stored in .bag file slightly earlier than scans, but should have the same timestamps as scans)

## Run
- To run trajectory and map optimization:
```console
$ roscd mad_ba/config
$ rosrun srrg2_executor auto_dl_finder
$ ln -s path_to_bag_file . # Soft link the .bag file to config folder
$ rosrun mad_ba main_app -c main_app.config
```
- To run trajectory and map optimization, and map evaluation
```
$ cd scripts
$ ./run_and_eval.sh
```

## Results

- Results will be stored in the `output` folder:
    - `eval` - contains results for map evaluation (if ```run_and_eval.sh``` was used)
    - `pcd`/`ply` - contains the subsequent optimization iterations of a map in
    - `tum` - contains the optimized trajectory
    - `scans` - contains scans recreated based on optimized map 


## Preparing initial trajectory
### Steps:
- Clone `loam-opensource`, `rosbag_player`, and `localize_main`
- Edit `loam_opensource_rosbag_player.launch` to set *.bag*paths etc
- Use ```$ roslaunch localize_mainloam_opensource_rosbag_player.launch```
- Trajectory must have the following coordinate frame: X - forward, Y - left, Z - Up

### Parameters

- `useRawSurfelOptimization_ = true` - optimizes surfels without solver; converges after 4-6 iterations (with `rawIterNum_=1`) instead of 3 but uses less RAM memory. Tested without noise and on 100 poses only.
- with `rawIterNum_=3` it converges after 3 outer iterations, or even slightly faster; setting it `rawIterNum_>3` do not give anything, sometimes it gets worse
- the same results are with added noise: `rawIterNum_=3` and `iterNum_=3` gives the best results, at least for tested 100 poses