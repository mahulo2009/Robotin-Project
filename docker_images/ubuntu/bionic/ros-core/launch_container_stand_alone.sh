docker run -it --rm \
	-e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/mhuertas/Work/Robotin-Project/robotin_project:/root/catkin_ws/src/robotin_project \
    -v /dev:/dev \
	--network=host \
	--privileged \
	ubuntu-melodic/ros:core



