#!/bin/sh

docker run -d  --runtime=nvidia \
	   --network=host \
	   --privileged \
     jetson/ros:melodic-robot-camera   

