#pragma once

#include <ros/ros.h>

#include "drive_parameters_source.h"
#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <vector>

constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_DRIVE_PARAMETERS_KEYBOARD = "input/drive_param/keyboard";
constexpr const char* TOPIC_DRIVE_PARAMETERS_JOYSTICK = "input/drive_param/joystick";
constexpr const char* TOPIC_DRIVE_PARAMETERS_AUTONOMOUS = "input/drive_param/autonomous";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";

/*
* This node subscribes to all publishers that send drive_param messages and selects one to forward to the car controller
*/
class DriveParametersMultiplexer
{
    public:
    /**
     * @brief Construct a new DriveParametersMultiplexer object and initialize sources for all
     * publishers of drive parameters
     */
    DriveParametersMultiplexer();

    private:
    ros::NodeHandle m_node_handle;

    std::array<std::unique_ptr<DriveParametersSource>, 3> m_sources;
    DriveParametersSource* m_last_updated_source;
    ros::Publisher m_drive_parameters_publisher;
    ros::Subscriber m_drive_mode_subscriber;

    DriveMode m_drive_mode;

    /**
     * @brief Determines wheter an updated source will be forwarded to the car controller,
     * based on which source was previously forwarded, whether they are idle or outdated.
     */
    bool validateSource(DriveParametersSource* source);

    /**
     * @brief This function should be called when a source has received a message.
     * It determines if the message should be forwarded and if it should, it sends the message
     * to the car controller.
     */
    void onUpdate(DriveParametersSource* source, const drive_msgs::drive_param::ConstPtr& message);

    void driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message);
};
