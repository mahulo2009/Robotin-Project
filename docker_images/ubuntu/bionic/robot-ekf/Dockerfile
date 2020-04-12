FROM ubuntu-melodic/ros:robot-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-robot-localization \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash", "-c", "source $CATKIN_WS/devel/setup.bash;roslaunch robotin_project ekf.launch"]
