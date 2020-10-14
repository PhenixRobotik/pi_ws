#include "can.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libsocketcan.h>

#include <sys/time.h>

#include <unistd.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

int start_can_iface()
{
  can_do_stop(CAN_IFACE);
  if(can_set_bitrate(CAN_IFACE, 500000) == -1)
  {
    printf("can't set can speed\n");
    return 0;
  }

  int is_can_started = can_do_start(CAN_IFACE);
  if(is_can_started == -1)
  {
    printf("can't start can\n");
    return 0;
  }
  else
  {
    printf("can started\n");
  }
  return 1;
}

int open_can_socket()
{
  int s;
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Socket");
    exit(1);
  }
  struct ifreq ifr;
  strcpy(ifr.ifr_name, CAN_IFACE);
  ioctl(s, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
   perror("Bind");
   exit(1);
  }

  return s;
}


static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
  (void) ins;
  return malloc(amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
  (void) ins;
  free(pointer);
}


CanardInstance start_canard()
{
  CanardInstance ins = canardInit(&memAllocate, &memFree);
  ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here we select Classic CAN.
  ins.node_id   = RPI_CAN_NODE_IP;

  return ins;
}


int can_frame_to_canard_rx(CanardInstance *pins, struct can_frame *pframe, CanardTransfer *ptransfer)
{
  const bool valid = ((pframe->can_id & CAN_EFF_FLAG) != 0) &&  // Extended frame
                           ((pframe->can_id & CAN_RTR_FLAG) == 0) &&  // Not RTR frame
                           ((pframe->can_id & CAN_ERR_FLAG) == 0);    // Not error frame
  //mute the error as described in platform specific
  if(!valid)
    return 1;

  CanardFrame received_frame;
  struct timeval tv;
  gettimeofday(&tv,NULL);
  received_frame.timestamp_usec = 1e+6 * tv.tv_sec + tv.tv_usec;
  //printf("ID RX : 0x%08x 0x%08x\n", pframe->can_id, pframe->can_id & CAN_EFF_MASK);
  received_frame.extended_can_id = pframe->can_id & CAN_EFF_MASK;
  received_frame.payload_size = pframe->can_dlc;
  received_frame.payload = pframe->data;
  const int8_t result = canardRxAccept(pins,
                                       &received_frame,            // The CAN frame received from the bus.
                                       0,  // If the transport is not redundant, use 0.
                                       ptransfer);
  return result;
}
