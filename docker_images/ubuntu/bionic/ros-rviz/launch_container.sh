docker run -it --rm --name rviz \
	-e DISPLAY=$DISPLAY \
       	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--network=host \
	--privileged \
	ubuntu/ros:melodic-desktop-full-bionic \
        rviz -d /root/catkin_ws/src/Robotin-Project/robotin_project/config/realsense.rviz	



