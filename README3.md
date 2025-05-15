## Create Docker Image and Container
- Build image: ```./build_image.sh```
- Create ontainer: ```./run_container.sh```

## Build MAD-BA
- Start and enter container: ```docker start mad_ba && docker exec -it mad_ba bash```
- Build ROS packages: ```cd /catkin_ws && catkin build -DCMAKE_BUILD_TYPE=Release```

## Prepare data
- Download .bag file ie.: 
```cd /root/share/dataset/VBR/Spagna/bag && bash download_spagna.sh```
- Preprocess .bag file: ```cd /root/share/dataset/VBR/Spagna && python3 process_bag.py```

## Run MAD-BA
- Start roscore in *screen*: ```screen -dmS roscore roscore```
- Run MAD-BA: ```roscd mad_ba && rosrun mad_ba  main_app -c config/VBR/spagna.config```

## Output
- Check **./docker_shared/output/** folder 