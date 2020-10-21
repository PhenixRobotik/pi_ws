#include "can2ros_templatization.h"
#include "can2ros_z.h"

std::map<CanardPortID, std::shared_ptr<CAN2ROS_interface>> can2ros_z;

static driver_data *pdata_ros_cb;

auto z_text = new CAN2ROS<std_msgs::String>  (can2ros_z, Z_TEXT_GET,    Z_TEXT_SET, "/z/text");
auto z_pump = new CAN2ROS<std_msgs::Bool>    (can2ros_z, Z_PUMP_GET,    Z_PUMP_SET, "/z/pump");
auto z_valve= new CAN2ROS<std_msgs::Bool>    (can2ros_z, Z_VALVE_GET,   Z_VALVE_SET,"/z/valve");
auto z_zpos = new CAN2ROS<std_msgs::Int32>   (can2ros_z, Z_ZPOS_GET,    Z_ZPOS_SET, "/z/position");
auto z_angle= new CAN2ROS<std_msgs::Int16>   (can2ros_z, Z_ANGLE_GET,   Z_ANGLE_SET,"/z/angle");
auto flaggy = new CAN2ROS<std_msgs::Bool>    (can2ros_z, FLAGGY_GET,    FLAGGY_SET, "/flaggy");
auto arm    = new CAN2ROS<std_msgs::Int16>   (can2ros_z, ARM_GET,       ARM_SET,    "/arm");
auto psensor= new CAN2ROS<std_msgs::Int32>   (can2ros_z, Z_PRESS_GET,   Z_PRESS_SET,"/z/pressure");
auto color_sensor= new CAN2ROS<std_msgs::ColorRGBA> (can2ros_z, Z_COLOR_GET,   Z_COLOR_SET,"/z/color");

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
