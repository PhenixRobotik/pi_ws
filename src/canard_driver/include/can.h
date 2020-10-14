#pragma once
#include "canard.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_IFACE "can0"
#define CAN_SPEED 500000
#define RPI_CAN_NODE_IP 43

int start_can_iface();
int open_can_socket();

CanardInstance start_canard();
int can_frame_to_canard_rx(CanardInstance *pins, struct can_frame *pframe, CanardTransfer *ptransfer);
