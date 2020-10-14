#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>

#include <sstream>

#include "can.h"

typedef struct{
  int can_socket;
  CanardInstance can_ins;
} driver_data;

void *RX_Thread_Func(void *vargp)
{
  driver_data *pdata = (driver_data *)vargp;
  while(1)
  {
    struct can_frame frame;
    int nbytes = read(pdata->can_socket, &frame, sizeof(struct can_frame));
    CanardTransfer transfer;
    int result = can_frame_to_canard_rx(&pdata->can_ins, &frame, &transfer);
    if(result == 1)
    {
      //printf("port_id:%d remote_node_id:%d size:%d\n", transfer.port_id, transfer.remote_node_id, transfer.payload_size);
      pdata->can_ins.memory_free(&pdata->can_ins, (void*)transfer.payload);
    }
    else if(result < 0)
    {
      ROS_ERROR("Canrd RX error!");
    }
  }
  return NULL;
}

int main(int argc, char **argv)
{
  if(!start_can_iface())
    return 1;

  driver_data data;
  data.can_socket = open_can_socket();
  data.can_ins = start_canard();

  CanardRxSubscription my_service_subscription;
  (void) canardRxSubscribe(&data.can_ins,
                         CanardTransferKindMessage,
                         1233,    // The Service-ID whose responses we will receive.
                         1024,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &my_service_subscription);
  pthread_t rx_thread_id;
  pthread_create(&rx_thread_id, NULL, RX_Thread_Func, &data);

  ros::init(argc, argv, "canard_driver");
  ros::NodeHandle n;

  ros::spin();


  return 0;
}
