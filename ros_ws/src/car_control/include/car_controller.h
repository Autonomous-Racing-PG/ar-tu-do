#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <time.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

constexpr const int MAX_SPEED = 5000;
constexpr const double MAX_ANGLE = 0.9;

class CarController
{
    public:
    CarController();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_drive_parameters_subscriber;
    ros::Subscriber m_command_subscriber;

    ros::Publisher m_speed_pulisher;
    ros::Publisher m_angle_publisher;
    ros::Publisher m_break_publisher;

    bool m_enabled;

    /**
     * @brief deals with incomming drive param messages
     *
     * @param parameters
     */
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    /**
     * @brief executes incomming commands
     *
     * @param command_message
     */
    void commandCallback(const std_msgs::String::ConstPtr& command_message);

    /**
     * @brief takes a speed and angle, converts and forwards them to gazebo/focbox
     *
     * @param raw_speed
     * @param raw_angle
     */
    void publishDriveParameters(double raw_speed, double raw_angle);
};
