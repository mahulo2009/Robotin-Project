FROM jetson/ros:melodic-perception


# INSTALL base packages
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# SETUP ROS workspace
ENV CATKIN_WS /root/catkin_ws
RUN mkdir -p $CATKIN_WS/src && \
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace $CATKIN_WS/src' && \	
	echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc 	

# INSTALL Realsense Lib
RUN sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 
RUN sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
RUN apt-get update && apt-get install -y --no-install-recommends \
	apt-utils \
	librealsense2-utils \
 	librealsense2-dev \
	&& \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# INSTALL ROS Realsense
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-ddynamic-reconfigure \ 
	ros-melodic-diagnostic-updater \
	&& \
     apt-get clean && \
     rm -rf /var/lib/apt/lists/*

# BUILD ROS Realsense
# This version uses librealsense v2.31.0
ENV REALSENSE_ROS_VERSION=2.2.11
RUN cd $CATKIN_WS/src && \		
	git clone https://github.com/IntelRealSense/realsense-ros.git && \	
	cd $CATKIN_WS/src/realsense-ros && \
	git checkout $REALSENSE_ROS_VERSION && \
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch realsense2_camera rs_camera.launch depth_fps:=15 infra_fps:=15 color_fps:=15 filters:=pointcloud"]




