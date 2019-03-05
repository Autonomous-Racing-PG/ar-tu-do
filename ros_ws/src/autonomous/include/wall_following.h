#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "drive_msgs/pid_input.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <ros/console.h>
#include <ros/ros.h>

constexpr const char* TOPIC_PID_INPUT = "/pid_input";
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

constexpr float DEG_TO_RAD = M_PI / 180.0;

class WallFollowing
{
    public:
    WallFollowing();

    private:
    std::array<float, 2> followWall(const sensor_msgs::LaserScan::ConstPtr& lidar);

    /**
     * @brief Checks if there is a wall to close in front of the car.
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);

    /**
     * @brief Maps the given angle to the corresponding index of the lidar array
     * and returns the range.
     */
    bool getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle, float& range);

    bool m_follow_right_wall = true;

    bool m_emergency_stop = true;

    float m_error = 0;
    float m_prev_error = 0;
    float m_corrected_angle = 0;
    float m_integral = 0;

    ros::NodeHandle m_node_handle;
    ros::Subscriber emer_stop_subscriber;
    ros::Subscriber lidar_subscriber;

    void emergencyStopCallback(const std_msgs::Bool emer_stop);

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);

    ros::Publisher pid_publisher;
};
