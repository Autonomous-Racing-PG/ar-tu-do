#ifndef REMOTE_KEYBOARD_H
#define REMOTE_KEYBOARD_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#include <signal.h>
#include <termios.h>
#include <time.h>

#define MODE "keyboard"

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/position"

#define TOPIC_STATUS_DMS "/status/dms"

#define KEYCODE_W 119
#define KEYCODE_A 97
#define KEYCODE_S 115
#define KEYCODE_D 100
#define KEYCODE_SPACE 32

#define DELTA_SPEED_UP 0.2
#define DELTA_SPEED_DOWN 0.1

#define DELTA_ANGLE_UP 0.2
#define DELTA_ANGLE_DOWN 0.1

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