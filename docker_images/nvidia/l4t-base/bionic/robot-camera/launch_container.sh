#!/bin/sh

docker run --runtime=nvidia --rm -it \
	   --network=host \
	   --privileged \
     jetson/ros:melodic-robot-camera   

