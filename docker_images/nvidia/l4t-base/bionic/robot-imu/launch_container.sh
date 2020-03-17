#!/bin/sh

docker run --runtime=nvidia --rm -it \
           --net=foo \
	   --env ROS_MASTER_URI=http://master:11311 \
           jetson/ros:melodic-robot-imu
