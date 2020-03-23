docker run -it --rm --name rviz \
	-e DISPLAY=$DISPLAY \
       	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--network=host \
	--privileged \
	ubuntu/ros:melodic-desktop-full-bionic \
        bash	



