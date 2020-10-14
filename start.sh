#!/bin/bash

source /home/ubuntu/.ros_config

rm -rf /home/ubuntu/.ros/log


roscore &

roslaunch rplidar_ros rplidar.launch --wait &


while true;
do
  sleep 1;
done
