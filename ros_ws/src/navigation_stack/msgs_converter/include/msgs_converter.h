#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief This converter class converts "cmd_vel" messages to "drive_param" messages.
 * The linear and angular velocity has to be transformed to linear velocity and steering angle.
 * TODO: Differential drive?
 */
class MSGSConverter
{
    public:
    /**
     * @brief Initializes the publisher and subscriber
     * 
     */
    MSGSConverter();

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

    constexpr VELOCITY_THRESHOLD = 0.001;
};