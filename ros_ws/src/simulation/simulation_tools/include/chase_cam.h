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

constexpr const char* TOPIC_GAZEBO_POSES =
    "/gazebo/racetrack/pose/info";

const ignition::math::Vector3d CAMERA_OFFSET(-1.7, 0, 0.5);

const std::string CAR_NAME = std::string("racer");

/**
 * @brief Chase camera Gazebo node that listens to racer pose messages
 * and updates the Gazebo client camera pose so that it follows the car,
 * similar to a racing video game
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
    gazebo::transport::SubscriberPtr m_racer_pose_subscriber;

    void gazeboPosesCallback(ConstPosesStampedPtr& message);
    void publishCameraPose(ignition::math::Pose3d& pose);
    void updateCamera(ignition::math::Pose3d& car_pose);

    ignition::math::Vector3d m_last_position;
    ignition::math::Quaterniond m_last_rotation;
};
