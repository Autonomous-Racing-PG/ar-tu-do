#pragma once

#include <ros/ros.h>

#include <chrono>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

constexpr const char* PARAMETER_DMS_CHECK_RATE = "dms_check_rate";
constexpr const char* PARAMETER_DMS_EXPIRATION = "dms_expiration";

// How old the last dead mans switch check can be, in seconds
constexpr auto DMS_EXPIRATION = std::chrono::duration<double>(0.1);

class DMSController
{
    public:
    int dms_check_rate; // How often the dead mans switch is checked. in Hz
    int dms_expiration; // How old the last dead mans switch check can be. in ms
    DMSController();
    void checkDMS();

    private:
    std::chrono::steady_clock::time_point m_last_dms_message_received;
    bool m_running = false;
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_dms_subscriber;
    ros::Publisher m_command_pulisher;

    /**
     * @brief checks the m_last_dms_message_received with the rate DMS_CHECK_RATE
     *
     * @param dms_message
     */
    void dmsCallback(const std_msgs::Int64::ConstPtr& dms_message);
};
