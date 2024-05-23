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
$ roscd structure_refinement/config
$ rosrun srrg2_executor auto_dl_finder
$ ln -s path_to_bag_file . # Soft link the .bag file to config folder
$ rosrun structure_refinement main_app -c main_app.config
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