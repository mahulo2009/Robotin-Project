FROM jetson/ros:melodic-base

# INSTALL robotin-navitagion dependencies
RUN apt-get update && apt-get install -y \
    ros-melodic-global-planner \
        ros-melodic-dwa-local-planner \
    ros-melodic-move-base \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# BUILD robotin-project
RUN cd $CATKIN_WS/src && \
        git clone https://github.com/mahulo2009/Robotin-Project.git && \
        cd $CATKIN_WS && \
        /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project navigation.launch"]

