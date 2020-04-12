#!/bin/sh

docker run -it --rm \
	--env ROS_MASTER_URI=http://gara-ubuntu:11311 \
	--network=host \
	ubuntu-melodic/ros:robot-imu \
	bash

