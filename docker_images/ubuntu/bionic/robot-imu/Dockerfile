FROM ubuntu-melodic/ros:robot-base

# INSTALL ROS serial & imu
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-imu-tools \
        vim \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# BUILD ROS lino msgs, imu calib and robotin project
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/linorobot/imu_calib.git && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project imu.launch"]
