#pragma once

#include "ai_config.h"

#include "floatfann.h"
#include "fann_cpp.h"

#include <dirent.h>
#include <map>

#include <ros/console.h>
#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"
#include <drive_msgs/drive_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

constexpr const char* PARAMETER_CONFIG_PATH = "nn_config_path";
constexpr const char* PARAMETER_DEFAULT_CONFIG = "nn_default_file";

constexpr const char* TOPIC_DRIVE_PARAMETERS_PUBLISH = "/commands/drive_param";
constexpr const char* TOPIC_LASER_SCAN_SUBSCRIBE = "/scan";


class AiDriver
{
    public:
    AiDriver();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_lidar_subscriber;
    ros::Publisher m_drive_parameter_publisher;
    ros::Timer m_timer;

    FANN::neural_net m_net;

    fann_type m_input[NUM_INPUT];
    // m_input[0] : current speed
    // m_input[1] : current wheel rotaton
    // m_input[2] : lidar 90 degrees right
    // m_input[3] : lidar 45 degrees right
    // m_input[4] : middle
    // m_input[5] : lidar 45 degrees left
    // m_input[6] : lidar 90 degrees left
    fann_type* m_output;
    // m_output[0] : next speed
    // m_output[1] : next wheel rotaton

    void timerCallback(const ros::TimerEvent&);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void publishDriveParameters(fann_type velocity, fann_type angle);

    void update();
};
