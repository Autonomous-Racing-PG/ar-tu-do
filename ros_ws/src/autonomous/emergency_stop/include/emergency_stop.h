#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "rviz_geometry_publisher.h"
#include <ros/console.h>
#include <ros/ros.h>

constexpr float DEG_TO_RAD = M_PI / 180.0;

constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/input/emergencystop";
constexpr const char* TOPIC_VISUALIZATION = "/emergencystop_visualization";

constexpr const char* LIDAR_FRAME = "laser";

constexpr float RANGE_THRESHOLD = 0.7;
constexpr const float CAR_BUMPER_LENGTH = 0.35;

constexpr float MAX_RANGE = 30;

enum class EmergencyStatus : int
{
    UNUSED = 0,
    ACTIVATED = 1,
    CLEARED = 2
};

class EmergencyStop
{
    public:
    EmergencyStop();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_lidar_subscriber;
    ros::Publisher m_emergency_stop_publisher;

    EmergencyStatus emergency_status;
    RvizGeometryPublisher m_debug_geometry;

    /**
     * @brief Returns true if there is a wall too close in front of the car.
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);


};