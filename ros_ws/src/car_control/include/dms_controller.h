#pragma once

#include <ros/ros.h>

#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#define TOPIC_COMMAND "/command"
#define TOPIC_DMS "/set/dms"

#define DMS_CHECK_RATE 20  // in Hz
#define DMS_EXPIRATION 100 // in ms

class DMSController
{
    public:
    DMSController();
    void checkDMS();

    private:
    long            last_dms_message_received;
    bool            running;
    ros::NodeHandle node_handle;
    ros::Subscriber dms_subscriber;
    ros::Publisher  command_pulisher;
    void dmsCallback(const std_msgs::Int64::ConstPtr& dms_message);
};
