#ifndef REMOTE_KEYBOARD_H
#define REMOTE_KEYBOARD_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#include <signal.h>
#include <termios.h>
#include <time.h>

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/angle"

#define KEYCODE_W 119
#define KEYCODE_A 97
#define KEYCODE_S 115
#define KEYCODE_D 100
#define KEYCODE_SPACE 32

class RemoteKeyboard
{
    public:
    RemoteKeyboard();
    void keyLoop();

    private:
    int getch();

    void adjustSpeed(double speed);
    void adjustAngle(double angle);

    ros::NodeHandle nh_;

    std::string input;

    ros::Publisher out_speed;
    ros::Publisher out_angle;
    ros::Publisher out_dms;

    double speed;
    double angle;
};

#endif
