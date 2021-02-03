#!/bin/sh

docker run -d --runtime=nvidia \
           --network=host \
	   --env ROS_MASTER_URI=http://nvidia:11311 \
	   jetson/ros:melodic-robot-imu
