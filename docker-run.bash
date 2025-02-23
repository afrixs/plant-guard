#!/bin/bash

mkdir -p ~/.plant_guard
mkdir -p ~/.plant_guard_cron
mkdir -p ~/.ros

docker run --init --rm --name plant_guard -it --privileged --net host \
  -v /etc/localtime:/etc/localtime:ro \
  -v ~/.plant_guard:/root/plant_guard:rw \
  -v ~/.plant_guard_cron:/var/spool/cron/crontabs:rw \
  -v ~/.ros:/root/.ros:rw \
  plant_guard_ros:app \
  bash -i -c "cron /var/spool/cron/crontabs/root ; r2pg && ros2 launch plantguard_bringup pump_crane.launch.py $@"