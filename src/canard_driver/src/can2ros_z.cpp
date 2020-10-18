#include "can2ros_templatization.h"
#include "can2ros_z.h"

std::map<CanardPortID, std::shared_ptr<CAN2ROS_interface>> can2ros_z;

driver_data *pdata_ros_cb;

auto z_valve= new CAN2ROS<std_msgs::Bool>    (can2ros_z, Z_VALVE_GET,   Z_VALVE_SET,"/z/valve");
auto z_text = new CAN2ROS<std_msgs::String>  (can2ros_z, Z_TEXT_GET,    Z_TEXT_SET, "/z/text");
auto z_pump = new CAN2ROS<std_msgs::Bool>    (can2ros_z, Z_PUMP_GET,    Z_PUMP_SET, "/z/pump");

void init_subscription_z(driver_data *pdata){
    pdata_ros_cb = pdata;
    for (auto &pair : can2ros_z) {
        pair.second->initialize(pdata);
    }
}

int decode2ros_z(driver_data *pdata, CanardTransfer *ptransfer) {
    auto it = can2ros_z.find(ptransfer->port_id);
    if (it != can2ros_z.end())
        return it->second->send_to_ros(ptransfer->payload_size, ptransfer->payload);
    else
        return 0;
}
