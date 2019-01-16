#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <functional>
#include <std_msgs/Float64.h>

class DriveParametersSource;
typedef std::function<void(DriveParametersSource*, const drive_msgs::drive_param::ConstPtr&)>
    DriveParameterCallbackFunction;

/*
*  This node subscribes to all publishers drive_param messages and selects one to forward to the car controller
*/
class DriveParametersSource
{
    public:
    DriveParametersSource(ros::NodeHandle* node_handle, const char* topic,
                          DriveParameterCallbackFunction update_callback, int priority, double timeout);

    bool isOutdated();
    bool isIdle();
    int getPriority();

    private:
    ros::Subscriber m_drive_parameters_subscriber;

    int m_priority;
    long int m_timeout;
    bool m_idle;
    long int m_last_update;

    DriveParameterCallbackFunction m_updateCallback;

    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};
