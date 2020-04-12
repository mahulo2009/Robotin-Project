FROM osrf/ros:melodic-desktop-full-bionic

# Install rtab-map for rviz
RUN apt-get update && apt-get install -y \
    ros-melodic-robot-localization \
    ros-melodic-rtabmap-ros \
	ros-melodic-ddynamic-reconfigure \
	ros-melodic-rqt-reconfigure \
	ros-melodic-teleop-twist-keyboard \
	ros-melodic-rosserial-arduino \
	ros-melodic-rosserial \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# SETUP ROS workspace
ENV CATKIN_WS /root/catkin_ws
RUN mkdir -p $CATKIN_WS/src && \
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace $CATKIN_WS/src' && \	
	echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc 

# BUILD lino msgs
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/linorobot/lino_msgs.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

