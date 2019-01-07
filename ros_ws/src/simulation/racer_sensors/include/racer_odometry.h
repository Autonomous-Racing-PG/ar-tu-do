#pragma once

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

/**
 * @brief Rostopic to which Gazebo publishes information about the models of the simulation.
 */
constexpr const char* TOPIC_GAZEBO_ODOM = "/gazebo/link_states";

/**
 * @brief Rostopic to which the odometry data should be published to.
 *
 */
constexpr const char* TOPIC_ODOM = "/odom";

/**
 * @brief Name of the link of the simulated car chassis.
 *
 */
constexpr const char* BASE_LINK_NAME = "racer::base_link";

class RacerOdometry
{
    public:
    /**
     * @brief Construcs a new Racer Odometry object.
     */
    RacerOdometry();

    private:
    /**
     * @brief ROS node handle, used for communication with the topics.
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief Subscribes to the data published from Gazebo.
     */
    ros::Subscriber m_gazebo_subscriber;

    /**
     * @brief Publishes the odometry data to the specified topic.
     */
    ros::Publisher m_odom_publisher;

    /**
     * @brief Contains the last received pose from Gazebo.
     */
    geometry_msgs::Pose m_last_received_pose;

    /**
     * @brief Contains the last received twist from Gazebo.
     */
    geometry_msgs::Twist m_last_received_twist;

    /**
     * @brief Contains the time of the last received message from Gazebo.
     */
    ros::Time m_last_received_stamp;

    /**
     * @brief Timer used for regulating the publish rate.
     */
    ros::Timer m_timer;

    /**
     * @brief Updates the held variables upon receiving new data from Gazebo.
     * @param links Contains information about all models in the Gazebo simulation.
     */
    void robotPoseUpdate(const gazebo_msgs::LinkStates::ConstPtr& links);

    /**
     * @brief Publishes the last received odometry data at a rate defined by the timer.
     */
    void timerCallback(const ros::TimerEvent&);
};