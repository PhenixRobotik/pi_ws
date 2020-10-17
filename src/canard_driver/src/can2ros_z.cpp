#include "can2ros_templatization.h"

std::shared_ptr<CAN2ROS_map> can2ros_z(new CAN2ROS_map({

    { Z_VALVE_GET,  std::make_shared<CAN2ROS<std_msgs::Bool>>   (Z_VALVE_GET,   Z_VALVE_SET,"/z/valve") },
    { Z_TEXT_GET,   std::make_shared<CAN2ROS<std_msgs::String>> (Z_TEXT_GET,    Z_TEXT_SET, "/z/text") },
    { Z_PUMP_GET,   std::make_shared<CAN2ROS<std_msgs::Bool>>   (Z_PUMP_GET,    Z_PUMP_SET, "/z/pump") },

}));

void init_subscription_z(driver_data *pdata, RemoteNodeMapping& remote_node_mapping){
    for (auto& pair : *can2ros_z) {
        pair.second->initialize(pdata);
    }
    remote_node_mapping[CAN_ID_Z] = can2ros_z;
}
