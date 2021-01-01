#!/bin/sh

docker run --runtime=nvidia --rm -it \
	   --network=host \
	   --privileged \
           --env ROS_MASTER_URI=http://nvidia:11311 \
           jetson/ros:melodic-robot-rplidar
