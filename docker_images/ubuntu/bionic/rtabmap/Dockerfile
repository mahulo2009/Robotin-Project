FROM ubuntu/ros:melodic-perception

# Install non-opencv dependent ros packages
RUN apt-get update && apt-get install --no-install-recommends -y \
	ros-melodic-ros-base \
	ros-melodic-image-transport \ 
	ros-melodic-tf \
	ros-melodic-tf-conversions \
	ros-melodic-eigen-conversions \
	ros-melodic-laser-geometry \
	ros-melodic-pcl-conversions \
	ros-melodic-pcl-ros \
	ros-melodic-move-base-msgs \
	ros-melodic-rviz \
	ros-melodic-octomap-ros \
	ros-melodic-move-base \
	libhdf5-openmpi-dev \
	libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/



# Install g2o
RUN apt-get update && apt-get install --no-install-recommends -y \
	ros-melodic-libg2o \
    && rm -rf /var/lib/apt/lists/*

# Install libpointmatcher
RUN apt-get update && apt-get install --no-install-recommends -y \
	libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ethz-asl/libnabo.git
RUN cd libnabo && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libnabo

RUN git clone https://github.com/ethz-asl/libpointmatcher.git
RUN cd libpointmatcher && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libpointmatcher

# Build RTAB-Map project

# Install rtabmap dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
	ros-melodic-rtabmap \
	ros-melodic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# UnInstall rtabmap dependencies
RUN apt-get update && apt-get remove  -y \
	ros-melodic-rtabmap \
	ros-melodic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# Clone source code
RUN git clone -b melodic-devel https://github.com/introlab/rtabmap.git

# Build rtabmap
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

#Intall ros packages
RUN cd $CATKIN_WS/src && \		
	git clone -b melodic https://github.com/ros-perception/vision_opencv vision_opencv && \
	git clone -b melodic-devel https://github.com/ros-perception/image_transport_plugins.git image_transport_plugins && \
	git clone -b melodic-devel https://github.com/introlab/rtabmap_ros.git rtabmap_ros && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'


