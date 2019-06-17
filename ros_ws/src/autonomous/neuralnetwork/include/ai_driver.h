#pragma once

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

namespace ai_driver
{
    constexpr const char* PARAMETER_CONFIG_FOLDER = "config_folder";
    constexpr const char* PARAMETER_CONFIG_FILE = "config_file";
    constexpr const char* PARAMETER_UPDATE_RATE = "update_rate";

    // publish
    constexpr const char* TOPIC_DRIVE_PARAMETERS = "/commands/drive_param";

    // subscribe
    constexpr const char* TOPIC_LASER_SCAN = "/scan";
    constexpr const char* TOPIC_NET_DEPLOY = "/ai/deploy";
}

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
    std::vector<fann_type> m_input;

    void timerCallback(const ros::TimerEvent&);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void netDeployCallback(const neuralnetwork::net_param::ConstPtr& data);
    void publishDriveParameters(float velocity, float angle);

    void update();
};
