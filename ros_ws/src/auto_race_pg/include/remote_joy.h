#ifndef REMOTE_JOY_H
#define REMOTE_JOY_H

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

#define MAX_REVERSE_SPEED 0.1

#define JOY_ANGLE_ANGULAR 0
#define JOY_ANGLE_LINEAR 1
#define JOY_R2 2

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/angle"

class RemoteJoy
{
    public:
    RemoteJoy();
    void keyLoop();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void publishAngle(double angle);
    void publishSpeed(double speed);

    ros::NodeHandle nh_;

    std::string input;

    ros::Publisher out_speed;
    ros::Publisher out_angle;

    ros::Subscriber in_joy;
};

#endif
