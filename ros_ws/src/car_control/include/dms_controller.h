#pragma once

#include <ros/ros.h>

#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

// How often the dead mans switch is checked. in Hz
constexpr const int DMS_CHECK_RATE = 20;

// How old the last dead mans swtich check can be. in ms
constexpr const int DMS_EXPIRATION = 100;

class DMSController
{
    public:
    DMSController();
    void checkDMS();

    private:
    long m_last_dms_message_received = 0;
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
