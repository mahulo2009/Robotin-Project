FROM osrf/ros:melodic-desktop-full-bionic

# Install rtab-map for rviz
RUN apt-get update && apt-get install -y \
        ros-melodic-rtabmap-ros \
	ros-melodic-ddynamic-reconfigure \
	ros-melodic-rqt-reconfigure \
	ros-melodic-teleop-twist-keyboard \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# SETUP ROS workspace
ENV CATKIN_WS /root/catkin_ws
RUN mkdir -p $CATKIN_WS/src && \
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace $CATKIN_WS/src' && \	
	echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc 

# BUILD ROS workspace
RUN cd $CATKIN_WS && \		
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

# BUILD ROS lino msgs
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/linorobot/lino_msgs.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

# BUILD ROS Robotin Project
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/mahulo2009/Robotin-Project.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'
