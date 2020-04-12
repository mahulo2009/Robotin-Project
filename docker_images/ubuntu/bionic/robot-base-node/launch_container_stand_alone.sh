docker run -it --rm \
	-e DISPLAY=$DISPLAY \
       	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--network=host \
	--privileged \
	ubuntu-melodic/ros:robot-base-node 




