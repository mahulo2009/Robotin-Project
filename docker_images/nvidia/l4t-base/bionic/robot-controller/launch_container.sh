#!/bin/sh

docker run --runtime=nvidia --rm -it \
           --net=foo \
	   --name master \
	   --privileged \
           jetson/ros:melodic-robot-controller
