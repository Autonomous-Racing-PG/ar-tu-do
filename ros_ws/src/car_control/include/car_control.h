#pragma once

#include <ros/ros.h>

#include <time.h>
#include <algorithm>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
#define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

#define TOPIC_DRIVE_PARAM "/set/drive_param"
#define TOPIC_COMMAND "/command"

#define MAX_SPEED 15000
#define MIN_SPEED 500
#define MAX_ANGLE 0.9

class CarController
{
    public:
    CarController();

    private:
    ros::NodeHandle nodeHandle;

    ros::Subscriber driveParametersSubscriber;
    ros::Subscriber deadMansSwitchSubscriber;
    ros::Subscriber commandSubscriber;
    
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    void command_callback(const std_msgs::String::ConstPtr& cmd);

    ros::Publisher speedPublisher;
    ros::Publisher anglePublisher;

    void publishDriveParameters(double rawSpeed, double rawAngle);

    bool run;
};
