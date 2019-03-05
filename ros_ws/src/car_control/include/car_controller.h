#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <time.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

constexpr const int MAX_SPEED = 5000;
constexpr const double MAX_ANGLE = 0.9;

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";
constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_UNLOCK_MOTOR = "/commands/unlock_motor";

class CarController
{
    public:
    CarController();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_drive_parameters_subscriber;
    ros::Subscriber m_unlock_motor_subscriber;

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
    void unlockMotorCallback(const std_msgs::Bool::ConstPtr& unlock_motor_message);

    /**
     * @brief takes a speed and angle, converts and forwards them to gazebo/focbox
     */
    void publishDriveParameters(double raw_speed, double raw_angle);

    /**
     * @brief publishes a brake message that stops the car
     */
    void stop();
};
