#include "can2ros_z.h"
#include "std_msgs/String.h"
#include "can.h"

static CanardRxSubscription out_z_subscription;//TODO put in data structure
static ros::Publisher out_pub;
static ros::Subscriber in_sub;
static driver_data *pdata_ros_cb;
static uint8_t in_z_transfer_id;

void z_in_Callback(const std_msgs::String::ConstPtr& msg)
{
  CanardTransfer transfer_tx = {
      .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
      .priority       = CanardPriorityNominal,
      .transfer_kind  = CanardTransferKindMessage,
      .port_id        = 1230,                       // This is the subject-ID.
      .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
      .transfer_id    = in_z_transfer_id,
      .payload_size   = msg->data.size(),
      .payload        = msg->data.c_str(),
  };
  ++in_z_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.

  int transfer_result = canard_transfer_to_can(pdata_ros_cb, &transfer_tx);
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

    out_pub.publish(msg);
    return 1;
  }
  return 0;
}

void init_subscription_z(driver_data *pdata)
{
  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         1233,    // The Service-ID whose responses we will receive.
                         1024,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &out_z_subscription);
  out_pub = pdata->pn->advertise<std_msgs::String>("/z/z_out", 1000);

  in_z_transfer_id = 0;
  pdata_ros_cb = pdata;
  in_sub = pdata->pn->subscribe("/z/z_in", 1000, z_in_Callback);
}
