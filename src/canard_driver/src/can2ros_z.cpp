#include "can2ros_z.h"
#include "can.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

//TODO put in data structure
static driver_data *pdata_ros_cb;

class ROS2CAN_interface {
public:
    virtual void initialize() = 0;
    virtual int send_to_ros(size_t payload_size, const void *payload) = 0;
};

template<class RosType> class ROS2CAN : public ROS2CAN_interface {
    public:
        ROS2CAN(
            std::map<CanardPortID, std::shared_ptr<ROS2CAN_interface>> &map_to_add_itself,
            uint16_t can_id_get,
            uint16_t can_id_set,
            std::string topic_name
        )
        : m_can_id_get(can_id_get)
        , m_can_id_set(can_id_set)
        , m_topic_name(topic_name)
        {
            map_to_add_itself[can_id_get] = std::shared_ptr<ROS2CAN_interface>(this);
        }

        ~ROS2CAN() = default;

        void initialize() {
            canardRxSubscribe(
                &pdata_ros_cb->can_ins,
                CanardTransferKindMessage,
                m_can_id_get,   // The Service-ID whose responses we will receive.
                1024,           // The extent (the maximum payload size); pick a huge value if not sure.
                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                &m_subscription
            );
            m_publisher = pdata_ros_cb->pn->advertise<RosType>(m_topic_name + "/get", 2);
            m_subscriber = pdata_ros_cb->pn->subscribe(m_topic_name + "/set", 2, &ROS2CAN<RosType>::send_to_can, this);
        }

        int send_to_ros(size_t payload_size, const void *payload) {
            RosType msg;
            can_to_msg(msg, payload_size, payload);
            m_publisher.publish(msg);
            return 1;
        }

    private:
        CanardRxSubscription m_subscription;
        ros::Publisher       m_publisher;
        ros::Subscriber      m_subscriber;

        uint8_t m_transfer_id = 0;

        const uint16_t    m_can_id_get;
        const uint16_t    m_can_id_set;
        const std::string m_topic_name;

        void send_to_can(RosType msg) {
            size_t payload_size = 0;
            void* payload;
            msg_to_can(msg, payload_size, payload);

            CanardTransfer transfer_tx = {
                .timestamp_usec = 0,                            // Zero if transmission deadline is not limited.
                .priority       = CanardPriorityNominal,
                .transfer_kind  = CanardTransferKindMessage,
                .port_id        = m_can_id_set,                 // This is the subject-ID.
                .remote_node_id = CANARD_NODE_ID_UNSET,         // Messages cannot be unicast, so use UNSET.
                .transfer_id    = m_transfer_id,
                .payload_size   = payload_size,
                .payload        = payload,
            };
            ++m_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.

            int transfer_result = canard_transfer_to_can(pdata_ros_cb, &transfer_tx);
        }

        void msg_to_can(RosType const& msg, size_t& payload_size, const void* payload);
        void can_to_msg(RosType& msg, size_t payload_size, const void* payload);
};

template<>
void ROS2CAN<std_msgs::String>::msg_to_can(std_msgs::String const& msg, size_t& payload_size, const void* payload) {
    payload_size = msg.data.size();
    payload = msg.data.c_str();
}
template<>
void ROS2CAN<std_msgs::String>::can_to_msg(std_msgs::String& msg, size_t payload_size, const void* payload) {
    std::string decoded((char *)payload);
    msg.data = decoded;
}


template<>
void ROS2CAN<std_msgs::Bool>::msg_to_can(std_msgs::Bool const& msg, size_t& payload_size, const void* payload) {
    payload_size = 1;
    payload = &msg.data;
}
template<>
void ROS2CAN<std_msgs::Bool>::can_to_msg(std_msgs::Bool& msg, size_t payload_size, const void* payload) {
    msg.data = ((unsigned char *)payload)[0];
}


std::map<CanardPortID, std::shared_ptr<ROS2CAN_interface>> can2ros_z;

ROS2CAN<std_msgs::String>   z_text  (can2ros_z, Z_TEXT_GET,    Z_TEXT_SET, "/z/text");
ROS2CAN<std_msgs::Bool>     z_pump  (can2ros_z, Z_PUMP_GET,    Z_PUMP_SET, "/z/pump");
ROS2CAN<std_msgs::Bool>     z_valve (can2ros_z, Z_VALVE_GET,   Z_VALVE_SET,"/z/valve");

int decode2ros_z(driver_data *pdata, CanardTransfer *ptransfer) {
    auto it = can2ros_z.find(ptransfer->port_id);
    if (it != can2ros_z.end())
        return it->second->send_to_ros(ptransfer->payload_size, ptransfer->payload);
    else
        return 0;
}

void init_subscription_z(driver_data *pdata){
    pdata_ros_cb = pdata;
    for (auto &pair : can2ros_z) {
        pair.second->initialize();
    }
}
