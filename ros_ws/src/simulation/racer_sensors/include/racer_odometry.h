#pragma once

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

#define TOPIC_GAZEBO_ODOM "/gazebo/link_states"
#define TOPIC_ODOM "/odom"

#define BASE_LINK_NAME "racer::base_link"

class RacerOdometry
{
    public:
    RacerOdometry();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber m_gazebo_subscriber;
    ros::Publisher m_odom_publisher;

    geometry_msgs::Pose m_last_received_pose;
    geometry_msgs::Twist m_last_received_twist;
    ros::Time m_last_received_stamp;

    ros::Timer m_timer;

    void robotPoseUpdate(const gazebo_msgs::LinkStates::ConstPtr& links);
    void timerCallback(const ros::TimerEvent&);
};