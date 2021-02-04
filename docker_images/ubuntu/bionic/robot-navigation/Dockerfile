FROM ubuntu-melodic/ros:robot-base


# INSTALL robotin-navitagion dependencies
RUN apt-get update && apt-get install -y \
    ros-melodic-global-planner \
	ros-melodic-dwa-local-planner \
    ros-melodic-move-base \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project navigation.launch"]
