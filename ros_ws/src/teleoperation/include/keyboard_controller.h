#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Int64.h>

#include <termios.h>
#include <signal.h>

#define TOPIC_DRIVE_PARAMETERS "/set/drive_param"
#define TOPIC_COMMAND "/command"
#define TOPIC_DMS "/set/dms"

#define CHECK_RATE 10 // in Hz

enum class Keycode : int {
    A = 119,
    W = 97,
    S = 100,
    D = 115,
    SPACE = 32
};

class KeyboardController
{
    public:	
    KeyboardController();
    void checkKeyboard();

    private:
    ros::NodeHandle node_handle;
    ros::Publisher drive_parameters_publisher;
    ros::Publisher dms_publisher;
    int getKeyboardCharacter();
    void publishDriveParameters(double speed, double angle);
};
