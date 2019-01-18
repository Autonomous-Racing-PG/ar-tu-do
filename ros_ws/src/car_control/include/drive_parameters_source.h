#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <functional>
#include <std_msgs/Float64.h>
#include <chrono>

class DriveParametersSource;
using DriveParameterCallbackFunction = std::function<void(DriveParametersSource*, const drive_msgs::drive_param::ConstPtr&)>;

constexpr double IDLE_RANGE = 0.01;

/*
*  A class that listens on a topic that publishes drive parameters and stores information about that source
*/
class DriveParametersSource
{
    public:
    /**
     * @brief Construct a new DriveParametersSource object,
     * subscribes to the topic and stores the parameters into local variables.
     *
     * @param node_handle A node handle is needed to create a topic subscriber
     * @param topic The name of the drive_param topic to subscribe to
     * @param update_callback Callback function that will be called whenever a message was received
     * @param priority Priority of the source. If multiple sources are not idle, the source with the
     * highest priority is forwarded.
     * @param timeout Messages will be deferred when they are older than this, in seconds.
     */
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

    /**
     * @brief Returns the priority of the source. If multiple sources are not idle, the source with the
     * highest priority is forwarded.
     */
    int getPriority();

    private:
    ros::Subscriber m_drive_parameters_subscriber;

    int m_priority;
    bool m_idle;
    std::chrono::duration<double> m_timeout;
    std::chrono::steady_clock::time_point m_last_update;

    DriveParameterCallbackFunction m_updateCallback;

    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
};
