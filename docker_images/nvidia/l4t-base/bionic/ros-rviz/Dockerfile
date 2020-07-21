FROM jetson/ros:melodic-core

# Install rtab-map for rviz
RUN apt-get update && apt-get install -y \
	ros-melodic-desktop \
        ros-melodic-rtabmap-ros \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# BUILD ROS lino msgs
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/linorobot/lino_msgs.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

