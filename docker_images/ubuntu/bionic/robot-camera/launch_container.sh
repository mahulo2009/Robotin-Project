#!/bin/sh

docker run --rm -it \
           --network=host \
	   --privileged \
           ubuntu/ros:melodic-robot-camera
