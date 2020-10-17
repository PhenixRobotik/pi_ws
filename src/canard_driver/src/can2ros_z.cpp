#include "can2ros_z.h"
#include "can.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

//TODO put in data structure
static driver_data *pdata_ros_cb;

static CanardRxSubscription out_z_subscription;
static ros::Publisher out_pub;
static ros::Subscriber in_sub;
static uint8_t in_z_transfer_id;

static CanardRxSubscription valve_status_z_subscription;
static ros::Publisher valve_pub;
static ros::Subscriber valve_sub;
static uint8_t valve_z_transfer_id;


void z_in_Callback(const std_msgs::String::ConstPtr& msg)
{
  CanardTransfer transfer_tx = {
      .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
      .priority       = CanardPriorityNominal,
      .transfer_kind  = CanardTransferKindMessage,
      .port_id        = Z_IN,                       // This is the subject-ID.
      .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
      .transfer_id    = in_z_transfer_id,
      .payload_size   = msg->data.size(),
      .payload        = msg->data.c_str(),
  };
  ++in_z_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.

  int transfer_result = canard_transfer_to_can(pdata_ros_cb, &transfer_tx);
}

void valve_z_Callback(const std_msgs::Bool::ConstPtr& msg)
{
  CanardTransfer transfer_tx = {
      .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
      .priority       = CanardPriorityNominal,
      .transfer_kind  = CanardTransferKindMessage,
      .port_id        = Z_PUMP_SET,                       // This is the subject-ID.
      .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
      .transfer_id    = valve_z_transfer_id,
      .payload_size   = 1,
      .payload        =  &msg->data,
  };
  ++valve_z_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.

  int transfer_result = canard_transfer_to_can(pdata_ros_cb, &transfer_tx);
}


int decode2ros_z(driver_data *pdata, CanardTransfer *ptransfer)
{
  if(ptransfer->port_id == Z_OUT)
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
  else if(ptransfer->port_id == Z_PUMP_GET)
  {
    std_msgs::Bool msg;
    if(ptransfer->payload_size != 1)
      return 0;
    msg.data = ((unsigned char *)ptransfer->payload)[0];

    valve_pub.publish(msg);
    return 1;
  }

  return 0;
}

void init_subscription_z(driver_data *pdata)
{
  pdata_ros_cb = pdata;

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         Z_OUT,    // The Service-ID whose responses we will receive.
                         1024,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &out_z_subscription);
  out_pub = pdata->pn->advertise<std_msgs::String>("/z/z_out", 2);
  in_z_transfer_id = 0;
  in_sub = pdata->pn->subscribe("/z/z_in", 2, z_in_Callback);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         Z_PUMP_GET,
                         1,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &valve_status_z_subscription);
  valve_pub = pdata->pn->advertise<std_msgs::String>("/z/pump_valve/status", 2);
  valve_z_transfer_id = 0;
  valve_sub = pdata->pn->subscribe("/z/pump_valve/set", 2, valve_z_Callback);
}
