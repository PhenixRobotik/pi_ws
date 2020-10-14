#pragma once

#include "ros/ros.h"
#include "canard.h"

typedef struct{
  int can_socket;
  CanardInstance can_ins;
  ros::NodeHandle *pn;
} driver_data;
