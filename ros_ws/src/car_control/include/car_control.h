#pragma once

#include <ros/ros.h>

#include <time.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>

#define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
#define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

#define TOPIC_DRIVE_PARAM "/set/drive_param"

#define MAX_SPEED 15000
#define MIN_SPEED 500
#define MAX_ANGLE 0.9

class CarControl
{
    public:
    CarControl();

    private:
    ros::NodeHandle nh_;

    ros::Subscriber in_drive_param;
    ros::Subscriber in_dms;

    void drive_param_callback(const drive_msgs::drive_param::ConstPtr& param);

    ros::Publisher out_speed;
    ros::Publisher out_angle;

    void adjustDriveParam(double speed, double angle);

    double speed;
    double angle;

    bool run;
};
