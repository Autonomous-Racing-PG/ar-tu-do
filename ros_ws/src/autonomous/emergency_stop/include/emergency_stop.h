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

#include <dynamic_reconfigure/server.h>
#include <emergency_stop/emergency_stopConfig.h>

constexpr float DEG_TO_RAD = M_PI / 180.0;

constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/input/emergencystop";
constexpr const char* TOPIC_VISUALIZATION = "/emergencystop_visualization";

constexpr const char* LIDAR_FRAME = "laser";

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
    dynamic_reconfigure::Server<emergency_stop::emergency_stopConfig> m_dyn_cfg_server;

    float m_range_threshold = 0.7;
    float m_car_bumper_length = 0.35;
    float m_max_range = 30;

    /**
     * @brief Returns true if there is a wall too close in front of the car.
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);

    void updateDynamicConfig();
};
