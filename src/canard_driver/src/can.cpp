#include "can.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libsocketcan.h>

#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "canard.h"

int start_can_iface()
{
  can_do_stop("can0");
  if(can_set_bitrate("can0", 500000) == -1)
  {
    printf("can't set can speed\n");
    return 0;
  }

  int is_can_started = can_do_start("can0");
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
