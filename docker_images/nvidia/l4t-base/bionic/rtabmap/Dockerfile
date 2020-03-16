FROM jetson/ros:realsense

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
# GTSAM
RUN git clone https://bitbucket.org/gtborg/gtsam.git
RUN cd gtsam && \
    mkdir build && \
    cd build && \
    cmake -DMETIS_SHARED=ON -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r gtsam

# G2O
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r g2o

RUN ln -sf /usr/lib/aarch64-linux-gnu/libGL.so.1.0.0 /usr/lib/aarch64-linux-gnu/libGL.so


# Install
RUN apt-get update && apt-get install --no-install-recommends -y \
	ros-melodic-rtabmap \
	ros-melodic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# Install
RUN apt-get update && apt-get remove  -y \
	ros-melodic-rtabmap \
	ros-melodic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# Clone source code
RUN git clone https://github.com/introlab/rtabmap.git

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Build RTAB-Map project
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
	git clone https://github.com/ros-perception/vision_opencv src/vision_opencv && \
	git clone https://github.com/ros-perception/image_transport_plugins.git src/image_transport_plugins && \
	git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros && \	
	cd $CATKIN_WS && \       
	/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'


