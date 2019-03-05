#pragma once

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

/**
 * @brief This converter class converts a 2D laser scan
 * as defined by sensor_msgs/LaserScan into a point
 * cloud as defined by sensor_msgs/PointCloud2.
 *
 * The main purpose of this class is the transformation
 * of the LaserScan to the "base_link" of the racer model.
 */
class LaserscanTransformer
{
    public:
    /**
     * @brief Initializes the publisher and subscriber
     *
     */
    LaserscanTransformer();

    private:
    /**
     * @brief ROS Handle of the class
     *
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief ROS Subscriber for sensor_msgs/LaserScan messages
     * TOPIC: "/racer/laser/scan"
     */
    ros::Subscriber laserscan_subscriber;

    /**
     * @brief ROS Publisher for the sensor_msgs/PointCloud2 message.
     * TOPIC: "/racer/laser/tf_pointcloud"
     */
    ros::Publisher pointcloud_publisher;

    /**
     * @brief This class will project laser scans into point clouds.
     */
    laser_geometry::LaserProjection projector;

    /**
     * @brief This class makes the task of receiving transforms easier
    */
    tf::TransformListener listener;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
};