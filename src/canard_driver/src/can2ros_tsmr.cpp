#include "can2ros_templatization.h"
#include "can2ros_tsmr.h"

std::map<CanardPortID, std::shared_ptr<CAN2ROS_interface>> can2ros_tsmr;

static driver_data *pdata_ros_cb;

auto tsmr_text   = new CAN2ROS<std_msgs::String>    (can2ros_tsmr, TSMR_TEXT_GET, TSMR_TEXT_SET, "/tsmr/text");
auto encoder_l   = new CAN2ROS<std_msgs::Int16>     (can2ros_tsmr, TSMR_ENCL_GET, TSMR_ENCL_SET, "/tsmr/encoder_l");
auto encoder_r   = new CAN2ROS<std_msgs::Int16>     (can2ros_tsmr, TSMR_ENCR_GET, TSMR_ENCR_SET, "/tsmr/encoder_r");
auto odom        = new CAN2ROS<nav_msgs::Odometry>  (can2ros_tsmr, TSMR_ODOM_GET, TSMR_ODOM_SET, "/odom");
auto power       = new CAN2ROS<std_msgs::Bool>      (can2ros_tsmr, TSMR_POWER_GET, TSMR_POWER_SET, "/tsmr/power");
auto translation = new CAN2ROS<std_msgs::Float32>   (can2ros_tsmr, TSMR_T_GET,    TSMR_T_SET, "/tsmr/translation");
auto translation_speed = new CAN2ROS<std_msgs::Float32>   (can2ros_tsmr, TSMR_TS_GET, TSMR_TS_SET, "/tsmr/translation_speed");
auto rotation = new CAN2ROS<std_msgs::Float32>   (can2ros_tsmr, TSMR_R_GET,    TSMR_R_SET, "/tsmr/rotation");
auto rotation_speed = new CAN2ROS<std_msgs::Float32>   (can2ros_tsmr, TSMR_RS_GET, TSMR_RS_SET, "/tsmr/rotation_speed");
auto xyt         = new CAN2ROS<geometry_msgs::Pose2D>(can2ros_tsmr, TSMR_XYT_GET, TSMR_XYT_SET, "/tsmr/xyt");
auto xyt_back    = new CAN2ROS<geometry_msgs::Pose2D>(can2ros_tsmr, TSMR_XYT_BACK_GET, TSMR_XYT_BACK_SET, "/tsmr/xyt_back");

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
