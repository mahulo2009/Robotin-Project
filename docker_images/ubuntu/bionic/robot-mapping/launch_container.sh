#!/bin/sh

docker run -it --rm \
	--network=host \
	ubuntu-melodic/ros:robot-mapping

