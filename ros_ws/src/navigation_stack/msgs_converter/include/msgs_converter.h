#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <geometry_msgs/Twist.h>


class MSGSConverter
{
    public:
    MSGSConverter();
    
    private:
    /**
     * @brief ROS Handle
     *
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief ROS Subscriber for servo position messages
     * TOPIC: "cmd_vel"
     */
    ros::Subscriber m_command_velocity_subscriber;

    /**
     * @brief ROS Publischer for the velocity of left rear wheel
     *
     */
    ros::Publisher m_to_car_control_publisher;

    void convertCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_message);
};