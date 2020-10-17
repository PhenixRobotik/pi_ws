#pragma once

#include <map>
#include <memory>

#include <canard.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "driver.h"
#include "can.h"

#include "can2ros_nodes.h"

/*****************************************************************************/
// Implementation template

template<class RosType> class CAN2ROS : public CAN2ROS_interface {
public:
    CAN2ROS(uint16_t can_id_get, uint16_t can_id_set, std::string topic_name)
    : m_can_id_get(can_id_get)
    , m_can_id_set(can_id_set)
    , m_topic_name(topic_name)
    { }

    ~CAN2ROS() = default;

    void initialize(driver_data* pdata_ros_cb) {
        m_driver_data = pdata_ros_cb;
        canardRxSubscribe(
            &pdata_ros_cb->can_ins,
            CanardTransferKindMessage,
            m_can_id_get,   // The Service-ID whose responses we will receive.
            1024,           // The extent (the maximum payload size); pick a huge value if not sure.
            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
            &m_subscription
        );
        m_publisher = m_driver_data->pn->advertise<RosType>(m_topic_name + "/get", 2);
        m_subscriber = m_driver_data->pn->subscribe(m_topic_name + "/set", 2, &CAN2ROS<RosType>::send_to_can, this);
    }

    int send_to_ros(size_t payload_size, const void *payload) {
        RosType msg;
        can_to_msg(msg, payload_size, payload);
        m_publisher.publish(msg);
        return 1;
    }

private:
    CanardRxSubscription    m_subscription;
    ros::Publisher          m_publisher;
    ros::Subscriber         m_subscriber;
    driver_data*            m_driver_data;

    uint8_t m_transfer_id = 0;

    const uint16_t    m_can_id_get;
    const uint16_t    m_can_id_set;
    const std::string m_topic_name;

    void send_to_can(RosType msg) {
        CanardTransfer transfer_tx = {
            .timestamp_usec = 0,                            // Zero if transmission deadline is not limited.
            .priority       = CanardPriorityNominal,
            .transfer_kind  = CanardTransferKindMessage,
            .port_id        = m_can_id_set,                 // This is the subject-ID.
            .remote_node_id = CANARD_NODE_ID_UNSET,         // Messages cannot be unicast, so use UNSET.
            .transfer_id    = m_transfer_id,
            .payload_size   = 0,
            .payload        = nullptr,
        };

        msg_to_can(msg, transfer_tx.payload_size, transfer_tx.payload);
        int transfer_result = canard_transfer_to_can(m_driver_data, &transfer_tx);
        ++m_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
    }

    void msg_to_can(RosType const& msg, size_t& payload_size, const void*& payload);
    void can_to_msg(RosType& msg, size_t payload_size, const void* payload);
};

/*****************************************************************************/
// Template specializations


template<>
void CAN2ROS<std_msgs::String>::msg_to_can(std_msgs::String const& msg, size_t& payload_size, const void*& payload) {
    payload_size = msg.data.size();
    payload = msg.data.c_str();
}
template<>
void CAN2ROS<std_msgs::String>::can_to_msg(std_msgs::String& msg, size_t payload_size, const void* payload) {
    std::string decoded((char *)payload, payload_size);
    msg.data = decoded;
}


template<>
void CAN2ROS<std_msgs::Bool>::msg_to_can(std_msgs::Bool const& msg, size_t& payload_size, const void*& payload) {
    payload_size = 1;
    payload = &msg.data;
}
template<>
void CAN2ROS<std_msgs::Bool>::can_to_msg(std_msgs::Bool& msg, size_t payload_size, const void* payload) {
    msg.data = ((unsigned char *)payload)[0];
}
