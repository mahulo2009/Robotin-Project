docker run -it --rm    \
	-e DISPLAY=$DISPLAY \
       	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--privileged \
	--network=host \
	--env ROS_MASTER_URI=http://gara-ubuntu:11311 \
	-v /home/mhuertas/Work/Robotin-Project/robotin_project:/root/catkin_ws/src/robotin_project \
	ubuntu/ros:melodic-robot-perception \
        python3 /root/catkin_ws/src/robotin_project/scripts/perception.py	



