#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <ros/console.h>
#include <ros/ros.h>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/wallfollowing";
constexpr const char* TOPIC_LASER_SCAN = "/scan";
constexpr const char* TOPIC_EMERGENCY_STOP = "/std_msgs/Bool";

constexpr int LIDAR_SAMPLE_COUNT = 720;

// Discard lidar measurements outside this range
constexpr float MIN_RANGE = 0.2;
constexpr float MAX_RANGE = 30;

constexpr float DEFAULT_RANGE = 4;

constexpr float SAMPLE_ANGLE_1 = 40;
constexpr float SAMPLE_ANGLE_2 = 90;
constexpr float SAMPLE_WINDOW_SIZE = SAMPLE_ANGLE_2 - SAMPLE_ANGLE_1;

constexpr float WALL_FOLLOWING_MAX_SPEED = 0.25;
constexpr float WALL_FOLLOWING_MIN_SPEED = 0.1;

// The car will aim to reach the target wall distance after travelling this distance
constexpr float PREDICTION_DISTANCE = 0.5;
// The desired distance between the wall and the car
constexpr float TARGET_WALL_DISTANCE = 0.5;

constexpr float DEG_TO_RAD = M_PI / 180.0;

class WallFollowing
{
    public:
    WallFollowing();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_emergency_stop_subscriber;
    ros::Subscriber m_lidar_subscriber;
    ros::Publisher m_drive_parameter_publisher;

    bool m_follow_right_wall = true;
    bool m_emergency_stop = true;

    float m_prev_error = 0;
    float m_integral = 0;
    
    void followWall(const sensor_msgs::LaserScan::ConstPtr& lidar);
    
    void emergencyStopCallback(const std_msgs::Bool emergency_stop_message);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);
    void publishDriveParameters(float velocity, float angle);

    /**
     * @brief Samples the lidar range at the given angle in degree.
     * If no valid sample was found, the range parameter is not changed and false is returned.
     * The angle 0 points forward.
     */
    bool getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle, float& range);
};
