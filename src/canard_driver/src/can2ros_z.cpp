#include "can2ros_z.h"
#include "std_msgs/String.h"

static CanardRxSubscription my_service_subscription;//TODO put in data structure
static ros::Publisher chatter_pub;

void init_subscription_z(driver_data *pdata)
{
  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         1233,    // The Service-ID whose responses we will receive.
                         1024,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &my_service_subscription);
  chatter_pub = pdata->pn->advertise<std_msgs::String>("chatter", 1000);
}

int decode2ros_z(driver_data *pdata, CanardTransfer *ptransfer)
{
  if(ptransfer->port_id == ID_DUMMY)
  {
    std_msgs::String msg;

    char string[128];
    for(int i = 0; i<ptransfer->payload_size; i++)
      string[i] = ((char *)ptransfer->payload)[i];
    string[ptransfer->payload_size] = '\0';
    msg.data = string;

    chatter_pub.publish(msg);
    return 1;
  }
  return 0;
}
