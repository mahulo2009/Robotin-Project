FROM jetson/ros:melodic-robot-base

# INSTALL robotin-project dependencies
RUN apt-get update && apt-get install -y \
        ros-melodic-rosserial-arduino \
        ros-melodic-rosserial \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project robot_controller.launch"]
