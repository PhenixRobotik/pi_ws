#!/bin/bash

source /home/ubuntu/.ros_config

rm -rf /home/ubuntu/.ros/log

sudo pigpiod
roscore &

screen -S tf -dm bash -c  "source /home/ubuntu/.ros_config; /home/ubuntu/pi_ws/frames.sh"
screen -S lidar -dm bash -c "source /home/ubuntu/.ros_config; roslaunch rplidar_ros rplidar.launch --wait"

screen -S hat -dm bash -c "source /home/ubuntu/.ros_config; rosrun hat_driver hat_driver.py"
screen -S can_driver -dm bash -c "sudo /home/ubuntu/pi_ws/start_can_node.sh"
screen -S odom_converter -dm bash -c "source /home/ubuntu/.ros_config; rosrun canard_driver odom_converter"

while true;
do
  sleep 1;
done
