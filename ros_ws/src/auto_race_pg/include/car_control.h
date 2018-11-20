#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <ros/ros.h>

#include <time.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
#define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/angle"

#define TOPIC_STATUS_MODE "/status/mode"
#define TOPIC_STATUS_DMS "/status/dms"

#define TIMER_DURATION 150 // ms
#define DMS_MAX 200        // ms

#define MAX_SPEED 15000
#define MIN_SPEED 500
#define MAX_ANGLE 0.8

class CarControl
{
    public:
    CarControl();

    private:
    ros::NodeHandle nh_;

    ros::Subscriber in_speed;
    ros::Subscriber in_angle;
    ros::Subscriber in_dms;

    void speed_callback(const std_msgs::Float64::ConstPtr& speed);
    void angle_callback(const std_msgs::Float64::ConstPtr& angle);
    void dms_callback(const std_msgs::Int64::ConstPtr& timestamp);

    ros::Publisher out_speed;
    ros::Publisher out_angle;
    ros::Publisher out_mode;

    void setMode(std::string mode);

    void adjustSpeed(double speed);
    void adjustAngle(double angle);

    bool run;
};

long dms = 0;

#endif // CAR_CONTROL_H