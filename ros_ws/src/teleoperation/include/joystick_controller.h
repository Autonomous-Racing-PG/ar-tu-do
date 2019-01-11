#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <sensor_msgs/Joy.h>

constexpr int JOYSTICK_AXIS_STEERING = 0;
constexpr int JOYSTICK_AXIS_THROTTLE = 5;
constexpr int JOYSTICK_AXIS_REVERSE = 2;
constexpr int JOYSTICK_BUTTON_DEADMANSSWITCH = 0;
constexpr int JOYSTICK_BUTTON_TOGGLE_INVERT_STEERING = 2;

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/set/drive_param";

constexpr const char* INVERT_STEERING_PARAMETER = "invert_steering";

class JoystickController
{
    public:
    JoystickController();

    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_drive_parameter_publisher;
    ros::Subscriber m_joystick_subscriber;

    bool m_invert_steering = false;

    bool m_toggle_invert_steering_state = false;

    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);
    void publishDriveParameters(double velocity, double steering_angle);
};
