#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "drive_msgs/pid_input.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <ros/console.h>
#include <ros/ros.h>

constexpr float WALL_FOLLOWING_MAX_SPEED = 0.25;
constexpr float WALL_FOLLOWING_MIN_SPEED = 0.1;
constexpr const char* TOPIC_PID_INPUT = "/pid_input";
constexpr const char* TOPIC_LASER_SCAN = "/racer/laser/scan";
constexpr const char* TOPIC_EMER_STOP = "/std_msgs/Bool";

class WallFollowing
{
    public:
    WallFollowing();

    private:
    /**
     * @brief Follows the wall by scanning the right side.
     *
     * @param lidar The lidar scan values
     * @return std::array<float, 2> The corrected velocity and angle values
     */
    std::array<float, 2> followRightWall(const sensor_msgs::LaserScan::ConstPtr& lidar);

    /**
     * @brief Follows the wall by scanning the left side.
     *
     * @param lidar The lidar scan values
     * @return std::array<float, 2> The corrected velocity and angle values
     */
    std::array<float, 2> followLeftWall(const sensor_msgs::LaserScan::ConstPtr& lidar);

    /**
     * @brief Checks if there is a wall to close in front of the car.
     *
     * @param lidar The lidar scan values
     * @return true ...if the car is too close to a wall
     * @return false ...if the car is not too close to a wall
     */
    bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar);

    /**
     * @brief Maps the given degree value to the corresponding array index of the lidar array
     * and returns the range.
     *
     * @param lidar The lidar scan values
     * @param theta Degree value
     * @return float Range at the given degree
     */
    float rangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float theta);

    /**
     * @brief Follow the right wall if true. Follow the left wall if false.
     */
    bool m_follow_right_wall = false;

    bool m_emergency_stop = true;

    float m_right_error = 0;
    float m_right_prev_error = 0;
    float m_right_corrected_angle = 0;
    float m_right_integral = 0;

    float m_left_error = 0;
    float m_left_prev_error = 0;
    float m_left_corrected_angle = 0;
    float m_left_integral = 0;

    ros::NodeHandle m_node_handle;
    ros::Subscriber emer_stop_subscriber;
    ros::Subscriber lidar_subscriber;

    /**
     * @brief The emergency stop callback method for this node. Gets the lidar input and handles the emergency stop.
     * 
     * @param emer_stop 
     */
    void emergencyStopCallback(const std_msgs::Bool emer_stop);

    /**
     * @brief The lidar callback method for this node. Gets the lidar input and handles the autonomous
     * controlloling.
     *
     * @param lidar
     */
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);

    ros::Publisher pid_publisher;
};
