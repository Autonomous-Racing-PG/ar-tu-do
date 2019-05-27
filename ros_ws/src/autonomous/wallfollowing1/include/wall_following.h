#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "pid_controller.h"
#include "rviz_geometry_publisher.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "wall.h"
#include <ros/console.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <wallfollowing1/wallfollowing1Config.h>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous";
constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/emergency_stop";
constexpr const char* TOPIC_VISUALIZATION = "/wallfollowing_visualization";

constexpr const char* LIDAR_FRAME = "laser";

constexpr float DEG_TO_RAD = M_PI / 180.0;

class WallFollowing
{
    public:
    WallFollowing();

    private:
    // Discard lidar measurements outside this range
    float m_min_range = 0.2;
    float m_max_range = 30;

    float m_fallback_range = 4;

    float m_sample_angle_1 = 30 * DEG_TO_RAD;
    float m_sample_angle_2 = 70 * DEG_TO_RAD;

    float m_wall_following_max_speed = 0.25;
    float m_wall_following_min_speed = 0.2;

    // The car will aim to reach the target wall distance after travelling this distance.
    // The higher this number, the more aggressively the car will cut corners.
    float m_prediction_distance = 2;
    // The desired distance between the wall and the car
    float m_target_wall_distance = 0.5;

    float m_time_between_scans = 0.025;

    ros::NodeHandle m_node_handle;

    dynamic_reconfigure::Server<wallfollowing1::wallfollowing1Config> m_dyn_cfg_server;

    ros::Subscriber m_lidar_subscriber;
    ros::Publisher m_drive_parameter_publisher;
    RvizGeometryPublisher m_debug_geometry;

    PIDController m_pid_controller = PIDController(5, 0.01, 0.2);

    void followSingleWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall);
    void followWalls(const sensor_msgs::LaserScan::ConstPtr& lidar);
    Wall getWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall);

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void publishDriveParameters(float velocity, float angle);

    void updateDynamicConfig();

    /**
     * @brief Samples the lidar range at the given angle in degree.
     * If no valid sample was found, FALLBACK_RANGE is returned.
     * The angle 0 points forward.
     */
    float getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle);
};
