#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Int64.h>

#include <signal.h>
#include <termios.h>
#include <time.h>

#define TOPIC_DRIVE_PARAMETERS "/set/drive_param"

#define KEYCODE_W 119
#define KEYCODE_A 97
#define KEYCODE_S 115
#define KEYCODE_D 100
#define KEYCODE_SPACE 32

class RemoteKeyboard
{
    public:
    RemoteKeyboard();
    void keyboardLoop();

    private:
    ros::NodeHandle node_handle;

    ros::Publisher drive_parameters_publisher;

    int getKeyboardCharacter();

    void publishDriveParameters(double speed, double angle);
};
