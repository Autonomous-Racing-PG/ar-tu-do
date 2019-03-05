#pragma once

#include <ros/ros.h>

#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

constexpr const char* PARAMETER_DMS_CHECK_RATE = "dms_check_rate";
constexpr const char* PARAMETER_DMS_EXPIRATION = "dms_expiration";

class DMSController
{
    public:
    DMSController();

    void spin();

    private:
    /**
     * @brief How often the unlock motor message is published, in Hz
     */
    int m_update_frequency;

    /**
     * @brief How old the last dead man's switch heartbeat can be, in ms
     */
    std::chrono::duration<double> m_expiration_time;

    std::chrono::steady_clock::time_point m_last_heartbeat_received;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_heartbeat_subscriber;
    ros::Publisher m_unlock_motor_publisher;

    void configureParameters();
    void publishUnlockMotor();

    void heartbeatCallback(const std_msgs::Int64::ConstPtr& dms_message);
};
