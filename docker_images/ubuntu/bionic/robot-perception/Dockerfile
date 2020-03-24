FROM ubuntu/ros:melodic-perception

RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
 
RUN add-apt-repository ppa:sweptlaser/python3-pcl && apt update
RUN apt-get update && apt-get install -y  \
	python3-pip \
	python3-yaml \
	python3-pcl \ 
	&& \
     apt-get clean && \
     rm -rf /var/lib/apt/lists/*

RUN pip3 install rospkg catkin_pkg


