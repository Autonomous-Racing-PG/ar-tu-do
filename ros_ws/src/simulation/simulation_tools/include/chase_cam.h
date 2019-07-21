#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

constexpr const char* TOPIC_CAMERA_POSE =
    "/gazebo/racetrack/user_camera/joy_pose";

/**
 * @brief 
 */
class ChaseCam
{
    public:
    ChaseCam();

    private:
    ros::NodeHandle m_ros_node_handle;
    ros::Publisher m_crash_publisher;

    gazebo::transport::NodePtr m_gazebo_node;
    gazebo::transport::PublisherPtr m_camera_pose_publisher;
};
