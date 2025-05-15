# BUILD THE IMAGE
ROS_IMAGE="mad_ba"
ROS_CONTAINER="mad_ba"

docker run -dit \
    --volume="./docker_shared:/root/share" \
    --privileged \
    --network=host \
    --name="$ROS_CONTAINER" \
    $ROS_IMAGE \
    /bin/bash

