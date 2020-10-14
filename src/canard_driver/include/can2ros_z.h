#pragma once

#include "driver.h"

#define CAN_ID_Z 42

#define ID_DUMMY 0x04D1

void init_subscription_z(driver_data *pdata);
int decode2ros_z(driver_data *pdata, CanardTransfer *ptransfer);
