FROM jetson/ros:melodic-core

# INSTALL ROS serial & imu
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-imu-tools \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# BUILD ROS lino msgs, imu calib and robotin project
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/linorobot/lino_msgs.git && \	
	git clone https://github.com/linorobot/imu_calib.git && \	
	git clone https://github.com/mahulo2009/Robotin-Project.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project imu.launch"]
