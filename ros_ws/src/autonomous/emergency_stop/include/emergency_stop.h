#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "rviz_geometry_publisher.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>
#include <ros/ros.h>

constexpr float DEG_TO_RAD = M_PI / 180.0;

constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/input/emergencystop";
constexpr const char* TOPIC_VISUALIZATION = "/emergencystop_visualization";

constexpr const char* LIDAR_FRAME = "laser";

constexpr const char* RANGE_THRESHOLD = "range_threshold";
constexpr const float RANGE_THRESHOLD_DEFAULT = 0.7;

constexpr const char* MAX_RANGE = "max_range";
constexpr const float MAX_RANGE_DEFAULT = 30.;

constexpr const float CAR_BUMPER_LENGTH = 0.35;

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

    EmergencyStatus m_emergency_status;
    RvizGeometryPublisher m_debug_geometry;

    float m_range_threshold;
    float m_max_range;

    void configureParameters();

    /**
     * @brief Returns true if there is a wall too close in front of the car.
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);
};