#include "can2ros_templatization.h"
#include "can2ros_tsmr.h"

std::map<CanardPortID, std::shared_ptr<CAN2ROS_interface>> can2ros_tsmr;

static driver_data *pdata_ros_cb;

auto tsmr_text = new CAN2ROS<std_msgs::String>  (can2ros_tsmr, TSMR_TEXT_GET, TSMR_TEXT_SET, "/tsmr/text");

void init_subscription_tsmr(driver_data *pdata){
    pdata_ros_cb = pdata;
    for (auto &pair : can2ros_tsmr) {
        pair.second->initialize(pdata);
    }
}

int decode2ros_tsmr(driver_data *pdata, CanardTransfer *ptransfer) {
    auto it = can2ros_tsmr.find(ptransfer->port_id);
    if (it != can2ros_tsmr.end())
        return it->second->send_to_ros(ptransfer->payload_size, ptransfer->payload);
    else
        return 0;
}
