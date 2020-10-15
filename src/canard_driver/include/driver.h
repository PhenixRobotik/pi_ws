#pragma once

#include "ros/ros.h"
#include "canard.h"

#include "can_defines.h"

typedef struct{
  int can_socket;
  CanardInstance can_ins;
  ros::NodeHandle *pn;
} driver_data;
