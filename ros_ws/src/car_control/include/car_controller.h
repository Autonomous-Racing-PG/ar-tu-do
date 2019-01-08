#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <time.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
#define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

#define TOPIC_DRIVE_PARAM "/set/drive_param"
#define TOPIC_COMMAND "/command"

class CarController
{
    public:
    CarController();

    private:
    ros::NodeHandle node_handle;

    ros::Subscriber drive_parameters_subscriber;
    ros::Subscriber command_subscriber;

    ros::Publisher speed_pulisher;
    ros::Publisher angle_publisher;

    bool enabled;

    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    void commandCallback(const std_msgs::String::ConstPtr& command_message);

    void publishDriveParameters(double raw_speed, double raw_angle);
};
