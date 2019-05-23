#pragma once

#include "drive_mode.h"
#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Time.h>

constexpr const char* PARAMETER_DMS_CHECK_RATE = "dms_check_rate";
constexpr const char* PARAMETER_DMS_EXPIRATION = "dms_expiration";
constexpr const char* PARAMETER_EMERGENCYSTOP_EXPIRATION = "emergencystop_expiration";
constexpr const char* PARAMETER_MODE_OVERRIDE = "mode_override";

constexpr const char* TOPIC_HEARTBEAT_MANUAL = "/input/heartbeat_manual";
constexpr const char* TOPIC_HEARTBEAT_AUTONOMOUS = "/input/heartbeat_autonomous";
constexpr const char* TOPIC_EMERGENCYSTOP = "/input/emergencystop";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";

constexpr DriveMode NO_OVERRIDE = DriveMode::LOCKED;

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

    DriveMode m_mode_override;

    /**
     * @brief How old the last dead man's switch heartbeat can be, in ms
     */
    ros::Duration m_expiration_time;
    /**
     * @brief How long the dead man's switch blocks all inputs, after receving an emergency stop message
     */
    ros::Duration m_emergencystop_expiration_time;

    ros::Time m_last_heartbeat_manual;
    ros::Time m_last_heartbeat_autonomous;
    ros::Time m_last_emergencystop;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_heartbeat_manual_subscriber;
    ros::Subscriber m_heartbeat_autonomous_subscriber;
    ros::Subscriber m_emergencystop_subscriber;
    ros::Publisher m_drive_mode_publisher;

    void configureParameters();
    void publishDriveMode();

    void heartbeatManualCallback(const std_msgs::Time::ConstPtr& message);
    void heartbeatAutonomousCallback(const std_msgs::Time::ConstPtr& message);
    void emergencystopCallback(const std_msgs::Time::ConstPtr& message);
    DriveMode getDriveMode();
};
