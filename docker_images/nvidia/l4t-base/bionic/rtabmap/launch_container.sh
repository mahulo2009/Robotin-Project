#!/bin/sh

docker run --runtime=nvidia --rm -it \
           --net=host \
	   --privileged \
           jetson/ros:rtabmap 
