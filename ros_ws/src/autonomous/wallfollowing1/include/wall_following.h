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

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous";
constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/emergency_stop";
constexpr const char* TOPIC_VISUALIZATION = "/wallfollowing_visualization";

constexpr const char* LIDAR_FRAME = "laser";

constexpr float DEG_TO_RAD = M_PI / 180.0;

// Discard lidar measurements outside this range
constexpr float MIN_RANGE = 0.2;
constexpr float MAX_RANGE = 30;

constexpr float FALLBACK_RANGE = 4;

constexpr float SAMPLE_ANGLE_1 = 30 * DEG_TO_RAD;
constexpr float SAMPLE_ANGLE_2 = 70 * DEG_TO_RAD;

constexpr float WALL_FOLLOWING_MAX_SPEED = 0.25;
constexpr float WALL_FOLLOWING_MIN_SPEED = 0.2;

// The car will aim to reach the target wall distance after travelling this distance.
// The higher this number, the more aggressively the car will cut corners.
constexpr float PREDICTION_DISTANCE = 2;
// The desired distance between the wall and the car
constexpr float TARGET_WALL_DISTANCE = 0.5;

constexpr float TIME_BETWEEN_SCANS = 0.025;

class WallFollowing
{
    public:
    WallFollowing();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_lidar_subscriber;
    ros::Publisher m_drive_parameter_publisher;
    RvizGeometryPublisher m_debug_geometry;

    PIDController m_pid_controller = PIDController(5, 0.01, 0.2);

    void followSingleWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall);
    void followWalls(const sensor_msgs::LaserScan::ConstPtr& lidar);
    Wall getWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall);

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void publishDriveParameters(float velocity, float angle);

    /**
     * @brief Samples the lidar range at the given angle in degree.
     * If no valid sample was found, FALLBACK_RANGE is returned.
     * The angle 0 points forward.
     */
    float getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle);
};
