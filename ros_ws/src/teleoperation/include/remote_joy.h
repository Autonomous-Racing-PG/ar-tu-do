#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <sensor_msgs/Joy.h>

#define MAX_REVERSE_SPEED 0.1

#define JOY_ANGLE_ANGULAR 0
#define JOY_ANGLE_LINEAR 1
#define JOY_R2 2

#define TOPIC_DRIVE_PARAM "/set/drive_param"

class RemoteJoy
{
    public:
    RemoteJoy();
    void keyLoop();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void publishDriveParam(double speed, double angle);

    ros::NodeHandle nh_;

    std::string input;

    ros::Publisher out_drive_param;

    ros::Subscriber in_joy;
};
