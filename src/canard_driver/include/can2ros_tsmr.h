#pragma once

#include "driver.h"

void init_subscription_tsmr(driver_data *pdata);
int decode2ros_tsmr(driver_data *pdata, CanardTransfer *ptransfer);
