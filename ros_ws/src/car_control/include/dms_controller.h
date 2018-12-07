#pragma once

#include <ros/ros.h>
#include <termios.h>

#include <std_msgs/String.h>

#define TOPIC_COMMAND "/command"

#define KEYCODE_SPACE 32

class DMSController
{
    public:
    DMSController();
    void keyLoop();

    private:
    ros::NodeHandle nh_;
	
    ros::Publisher out_cmd_;
	
    int getch();
};
