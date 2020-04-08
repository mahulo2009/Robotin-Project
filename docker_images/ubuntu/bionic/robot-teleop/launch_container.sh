docker run -it --rm \
	-e DISPLAY=$DISPLAY \
       	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/mhuertas/Work/Robotin-Project/robotin_project:/root/catkin_ws/src/robotin_project \
	--env ROS_MASTER_URI=http://gara-ubuntu:11311 \
	--network=host \
	--privileged \
	ubuntu/ros:melodic-robot-teleop \
	bash





