#pragma once

#include "drive_mode.h"
#include <ros/ros.h>

#include <algorithm>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";
constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";

class CarController
{
    public:
    CarController();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_drive_parameters_subscriber;
    ros::Subscriber m_drive_mode_subscriber;

    ros::Publisher m_speed_pulisher;
    ros::Publisher m_angle_publisher;
    ros::Publisher m_brake_publisher;

    bool m_motor_unlocked;

    /**
     * @brief deals with incomming drive param messages
     */
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    /**
     * @brief callback for the topic that enables / disables the motor
     */
    void driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message);
    /**
     * @brief takes a speed and angle, converts and forwards them to gazebo/focbox
     */
    void publishDriveParameters(double raw_speed, double raw_angle);

    /**
     * @brief publishes a brake message that stops the car
     */
    void stop();
};
