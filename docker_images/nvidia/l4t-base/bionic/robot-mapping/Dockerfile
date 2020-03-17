FROM jetson/ros:melodic-rtabmap

# BUILD ROS Robotin Project
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/mahulo2009/Robotin-Project.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project mapping.launch"]
