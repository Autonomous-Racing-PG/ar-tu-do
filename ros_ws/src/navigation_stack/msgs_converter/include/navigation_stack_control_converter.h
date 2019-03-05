#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <geometry_msgs/Twist.h>

constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* CMD_VEL = "cmd_vel";

/**
 * @brief This converter class converts "cmd_vel" messages to "drive_param" messages.
 * The linear and angular velocity has to be transformed to linear velocity and steering angle.
 * TODO: Differential drive?
 */
class NavigationStackControlConverter
{
    public:
    /**
     * @brief Initializes the publisher and subscriber
     *
     */
    NavigationStackControlConverter();

    private:
    /**
     * @brief ROS Handle of the class
     *
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief ROS Subscriber for servo position messages
     * TOPIC: "cmd_vel"
     */
    ros::Subscriber m_command_velocity_subscriber;

    /**
     * @brief ROS Publisher for the drive param message.
     *
     */
    ros::Publisher m_drive_param_publisher;

    void convertCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_message);
};