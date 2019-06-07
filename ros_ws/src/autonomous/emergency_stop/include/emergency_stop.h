#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/console.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <emergency_stop/emergency_stopConfig.h>

constexpr float DEG_TO_RAD = M_PI / 180.0;

constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/input/emergencystop";

class EmergencyStop
{
    public:
    EmergencyStop();

    private:
    /**
     * @brief Returns true if there is a wall too close in front of the car.
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);

    dynamic_reconfigure::Server<emergency_stop::emergency_stopConfig> m_dyn_cfg_server;

    float m_range_threshold = 0.7;
    float m_car_bumper_length = 0.2;
    float m_max_range = 30;

    ros::NodeHandle m_node_handle;
    ros::Subscriber m_lidar_subscriber;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);

    void updateDynamicConfig();

    ros::Publisher m_emergency_stop_publisher;
};
