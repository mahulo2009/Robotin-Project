FROM jetson/ros:melodic-base

# INSTALL robotin-project dependencies
RUN apt-get update && apt-get install -y \
		ros-melodic-tf2-geometry-msgs \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# BUILD lino msgs
RUN cd $CATKIN_WS/src && \
        git clone https://github.com/linorobot/lino_msgs.git && \
        cd $CATKIN_WS && \
        /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

# BUILD robotin-project
RUN cd $CATKIN_WS/src && \
        git clone https://github.com/mahulo2009/Robotin-Project.git && \
        cd $CATKIN_WS && \
        /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release'

