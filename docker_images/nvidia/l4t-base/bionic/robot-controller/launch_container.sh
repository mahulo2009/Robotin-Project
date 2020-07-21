#!/bin/sh

docker run --runtime=nvidia --rm -it \
	   --privileged \
	   --network=host \
	   --env ROS_MASTER_URI=http://nvidia:11311 \
           jetson/ros:melodic-robot-controller
