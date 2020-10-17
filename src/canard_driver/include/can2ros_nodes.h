#pragma once

#include <map>
#include <functional>

#include "driver.h"

class CAN2ROS_interface {
public:
    virtual void initialize(driver_data*) = 0;
    virtual int send_to_ros(size_t payload_size, const void *payload) = 0;
};

using CAN2ROS_map = std::map<CanardPortID, std::shared_ptr<CAN2ROS_interface>>;

using RemoteNodeMapping = std::map<CanardNodeID, std::shared_ptr<CAN2ROS_map>>;


void init_subscription_z(driver_data *pdata, RemoteNodeMapping& remote_node_mapping);
int decode2ros_z(CanardTransfer *ptransfer);
