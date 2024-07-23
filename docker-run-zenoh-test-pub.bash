#!/bin/bash

docker run --init --rm --name plant_guard -it --privileged --net=host \
  plant_guard_ros:app \
  bash -c "ros2 launch plantguard_bringup zenoh_test_pub.launch.py"