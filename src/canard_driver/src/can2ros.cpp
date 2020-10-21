#include "can2ros.h"

#include "can2ros_tsmr.h"
#include "can2ros_z.h"

void init_subscription(driver_data *pdata) {
  init_subscription_tsmr(pdata);
  init_subscription_z(pdata);
}

int decode2ros(driver_data *pdata, CanardTransfer *ptransfer)
{
  switch (ptransfer->remote_node_id) {
    case CAN_ID_TSMR: return decode2ros_tsmr(pdata, ptransfer);
    case CAN_ID_Z: return decode2ros_z(pdata, ptransfer);


    default: return 0;
  }

}
