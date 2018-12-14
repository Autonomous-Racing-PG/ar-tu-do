#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <sensor_msgs/Joy.h>

#include <std_msgs/Int64.h>

#define JOYSTICK_AXIS_STEERING 0
#define JOYSTICK_AXIS_THROTTLE 5
#define JOYSTICK_BUTTON_DEADMANSSWITCH 0

#define TOPIC_DRIVE_PARAMETERS "/set/drive_param"
#define TOPIC_DMS "/set/dms"

class JoystickController
{
    public:
    JoystickController();

    private:
    ros::NodeHandle node_handle;
    ros::Publisher drive_parameter_publisher;
    ros::Publisher dms_publisher;
    ros::Subscriber joystick_subscriber;

    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);
    void publishDriveParameters(double velocity, double steering_angle);
};
