#pragma once

#include "driver.h"

//init sub for can rx and the associated publisher
void init_subscription(driver_data *pdata);
int decode2ros(driver_data *pdata, CanardTransfer *ptransfer);
