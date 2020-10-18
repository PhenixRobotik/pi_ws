#!/bin/bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_link 100 &

rosrun tf static_transform_publisher 0 0 0 0 3.14 0 base_link laser 100 &

while true
do
  sleep 1
done
