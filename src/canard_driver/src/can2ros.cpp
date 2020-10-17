#include "can2ros.h"

#include "can2ros_nodes.h"

RemoteNodeMapping remote_node_mapping;

void init_subscription(driver_data *pdata) {
    init_subscription_z(pdata, remote_node_mapping);
}

int decode2ros(driver_data *pdata, CanardTransfer *ptransfer) {
    auto mapping = remote_node_mapping.find(ptransfer->port_id);
    if (mapping != remote_node_mapping.end()) {
        auto channel = mapping->second->find(ptransfer->port_id);
        if (channel != mapping->second->end())
            return channel->second->send_to_ros(ptransfer->payload_size, ptransfer->payload);
    }
    return 0;
}
