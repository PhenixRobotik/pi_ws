#pragma once

#include "driver.h"

void init_subscription_z(driver_data *pdata);
int decode2ros_z(driver_data *pdata, CanardTransfer *ptransfer);
