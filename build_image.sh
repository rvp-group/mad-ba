xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="mad_ba"
if [ -z "$1" ]
	then
		echo "RUNNING WITH 1 WORKER"
		WORKERS=1
	else
		echo "RUNNING WITH $1 WORKERS"
		WORKERS=$1
fi
docker build --no-cache -f Dockerfile -t $ROS_IMAGE ./.
