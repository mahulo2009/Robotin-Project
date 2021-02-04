#!/bin/sh

docker run -it --rm \
	--env ROS_MASTER_URI=http://nvidia:11311 \
	--network=host \
	ubuntu-melodic/ros:robot-navigation bash

