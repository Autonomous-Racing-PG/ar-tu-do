#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <functional>
#include <std_msgs/Float64.h>

class DriveParametersSource;
typedef std::function<void(DriveParametersSource*, const drive_msgs::drive_param::ConstPtr&)>
    DriveParameterCallbackFunction;

/*
*  A class that listens on a topic that publishes drive parameters and stores information about that source
*/
class DriveParametersSource
{
    public:
    DriveParametersSource(ros::NodeHandle* node_handle, const char* topic,
                          DriveParameterCallbackFunction update_callback, int priority, double timeout);

    /**
     * @brief Returns true if no update was received for a certain time, determined by the timeout variable.
     */
    bool isOutdated();

    /**
     * @brief Returns true if the last update contained values close to 0 for both steering and throttle.
     * If no update was received yet, this returns true.
     */
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
