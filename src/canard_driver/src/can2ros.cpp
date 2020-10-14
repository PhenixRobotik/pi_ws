#include "can2ros.h"

#include "can2ros_z.h"

void init_subscription(driver_data *pdata)
{
  init_subscription_z(pdata);
}

int decode2ros(driver_data *pdata, CanardTransfer *ptransfer)
{
  if(ptransfer->remote_node_id == CAN_ID_Z)
    return decode2ros_z(pdata, ptransfer);
  return 0;
}
