FROM jetson/ros:melodic-core

# INSTALL ROS rplidar
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-rplidar-ros \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash", "-c", "roslaunch rplidar_ros rplidar.launch"]
