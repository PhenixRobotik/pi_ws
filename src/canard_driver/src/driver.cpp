#include "driver.h"

#include <pthread.h>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "can.h"
#include "can2ros.h"

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
      decode2ros(pdata, &transfer);
      pdata->can_ins.memory_free(&pdata->can_ins, (void*)transfer.payload);
    }
    else if(result < 0)
      ROS_ERROR("Canard RX error!");
  }
  return NULL;
}

int main(int argc, char **argv)
{
  if(!start_can_iface()) {
    // return 1;
  }

  driver_data data;
  data.can_socket = open_can_socket();
  data.can_ins = start_canard();

  ros::init(argc, argv, "canard_driver");
  ros::NodeHandle n;
  data.pn = &n;

  init_subscription(&data);

  pthread_t rx_thread_id;
  pthread_create(&rx_thread_id, NULL, RX_Thread_Func, &data);

  ros::spin();


  return 0;
}
