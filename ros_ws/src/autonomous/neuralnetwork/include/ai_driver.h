#pragma once

#include "ai_config.h"

// http://leenissen.dk/fann/html/files/fann_cpp-h.html
// clang-format off
#include "floatfann.h"
#include "fann_cpp.h"
// clang-format on

#include <dirent.h>
#include <map>

#include <ros/console.h>
#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"
#include <drive_msgs/drive_param.h>
#include <neuralnetwork/net_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

constexpr const char* PARAMETER_CONFIG_FOLDER = "config_folder";
constexpr const char* PARAMETER_CONFIG_FILE = "config_file";
constexpr const char* PARAMETER_UPDATE_RATE = "update_rate";

constexpr const char* TOPIC_DRIVE_PARAMETERS_PUBLISH = "/commands/drive_param";
constexpr const char* TOPIC_LASER_SCAN_SUBSCRIBE = "/scan";

constexpr const char* TOPIC_NET_DEPLOY_SUBSCRIBE = "/ai/deploy";

class AiDriver
{
    public:
    AiDriver();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_lidar_subscriber;
    ros::Subscriber m_net_deploy_subscriber;
    ros::Publisher m_drive_parameters_publisher;
    ros::Timer m_timer;
    double m_update_rate;
    bool m_deployed;

    FANN::neural_net m_net;

    int m_changes_lidar = 0;
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
    void netDeployCallback(const neuralnetwork::net_param::ConstPtr& data);
    void publishDriveParameters(fann_type velocity, fann_type angle);

    void update();
};
