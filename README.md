<div align="center">
    <h1>MAD-BA</h1>
    <h3>3D LiDAR Bundle Adjustment &mdash; from Uncertainty Modelling to Structure Optimization</h3>
</div>

[![MAD-BA](http://img.youtube.com/vi/Uz1ST_lP8r4/0.jpg)](http://www.youtube.com/watch?v=Uz1ST_lP8r4 "MAD-BA: 3D LiDAR Bundle Adjustment")
<iframe width="560" height="315" src="https://www.youtube.com/embed/Uz1ST_lP8r4" frameborder="0" allowfullscreen></iframe>


### Create Docker Image and Container
- Build image and create container: ```./build_and_run.sh```

### Build MAD-BA
- Start and enter container: ```docker start mad_ba && docker exec -it mad_ba bash```
- Build ROS packages: ```cd /catkin_ws && catkin build -DCMAKE_BUILD_TYPE=Release```

### Prepare data
- Download .bag file ie.: 
```cd /root/share/dataset/VBR/Spagna/bag && bash download_spagna.sh```
- Preprocess .bag file: ```cd /root/share/dataset/VBR/Spagna && python3 process_bag.py```

### Run MAD-BA
- Start roscore in *screen*: ```screen -dmS roscore roscore```
- Run MAD-BA: ```roscd mad_ba && rosrun mad_ba  main_app -c config/VBR/spagna.config```

### Output
- Check **./docker_shared/output/** folder 