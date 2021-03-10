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


