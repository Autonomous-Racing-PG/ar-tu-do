#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "drive_msgs/pid_input.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <ros/console.h>
#include <ros/ros.h>

constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMER_STOP = "/std_msgs/Bool";

class EmergencyStop
{
    public:
    EmergencyStop();

    private:
    /**
     * @brief Checks if there is a wall to close in front of the car.
     *
     * @param lidar The lidar scan values
     * @return true ...if the car is too close to a wall
     * @return false ...if the car is not too close to a wall
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);

    /**
     * @brief Maps the given degree value to the corresponding array index of the lidar array
     * and returns the range.
     *
     * @param lidar The lidar scan values
     * @param theta Degree value
     * @return float Range at the given degree
     */
    float rangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float theta);

    ros::NodeHandle m_node_handle;
    ros::Subscriber lidar_subscriber;

    /**
     * @brief The callback method for this node. Gets the lidar input and handles the emergency stop.
     *
     * @param lidar
     */
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);

    ros::Publisher emer_stop_publisher;
};