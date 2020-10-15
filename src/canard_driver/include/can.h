#pragma once
#include "canard.h"
#include "driver.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_IFACE "can0"

int start_can_iface();
int open_can_socket();

CanardInstance start_canard();
int can_frame_to_canard_rx(CanardInstance *pins, struct can_frame *pframe, CanardTransfer *ptransfer);
int canard_transfer_to_can(driver_data *pdata, CanardTransfer *ptransfer_tx);
