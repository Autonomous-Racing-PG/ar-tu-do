#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Int64.h>

#include <signal.h>
#include <termios.h>
#include <time.h>

#define TOPIC_DRIVE_PARAMETERS "/set/drive_param"

enum class Keycode : int
{
    W     = 119,
    A     = 97,
    S     = 115,
    D     = 100,
    SPACE = 32
};

class KeyboardController
{
    public:
    KeyboardController();
    void keyboardLoop();

    private:
    ros::NodeHandle node_handle;

    ros::Publisher drive_parameters_publisher;

    int getKeyboardCharacter();

    void publishDriveParameters(double speed, double angle);
};
