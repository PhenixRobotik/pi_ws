#!/bin/bash

source /home/ubuntu/.ros_config

rm -rf /home/ubuntu/.ros/log


roscore &

screen -S lidar -dm bash -c "source /home/ubuntu/.ros_config; roslaunch rplidar_ros rplidar.launch --wait"

screen -S can_driver -dm bash -c "sudo /home/ubuntu/pi_ws/start_can_node.sh"

while true;
do
  sleep 1;
done
