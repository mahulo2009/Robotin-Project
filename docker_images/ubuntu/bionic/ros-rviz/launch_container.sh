docker run -it --rm \
	-e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
	--env ROS_MASTER_URI=http://nvidia:11311 \
	--network=host \
	--privileged \
	ubuntu-melodic/ros:ros-rviz  



