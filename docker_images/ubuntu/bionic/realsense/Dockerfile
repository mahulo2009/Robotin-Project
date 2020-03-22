FROM ubuntu/ros:melodic-perception

# INSTALL base packages
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# INSTALL Realsense Lib
RUN apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  && add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
RUN apt-get update && apt-get install -y  \
	apt-utils \
	librealsense2-utils \
 	librealsense2-dev \
	&& \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# INSTALL ROS Realsense
RUN apt-get update && apt-get install -y  \
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





