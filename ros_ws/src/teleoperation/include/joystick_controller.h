#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <sensor_msgs/Joy.h>

#include <std_msgs/Int64.h>

constexpr unsigned JOYSTICK_AXIS_STEERING = 0;
constexpr unsigned JOYSTICK_AXIS_THROTTLE = 5;
constexpr unsigned JOYSTICK_BUTTON_DEADMANSSWITCH = 0;

#define TOPIC_DRIVE_PARAMETERS "/set/drive_param"
#define TOPIC_DEAD_MANS_SWITCH "/set/dms"
#define TOPIC_JOY "/joy"

class JoystickController
{
    public:
    JoystickController();

    private:
    ros::NodeHandle node_handle;

    ros::Publisher m_drive_parameter_publisher;
    ros::Publisher m_dead_mans_switch_publisher;

    ros::Subscriber m_joystick_subscriber;

    ros::Timer m_joystick_connection_timer;

    bool m_joystick_connected;

    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);
    void publishDriveParameters(double velocity, double steering_angle);
    void joystickConnectionCallback(const ros::TimerEvent& event);
};
