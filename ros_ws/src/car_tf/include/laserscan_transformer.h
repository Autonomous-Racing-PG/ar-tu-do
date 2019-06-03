#pragma once

#include <ros/ros.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#pragma GCC diagnostic pop

constexpr const char* TOPIC_LASER_SCAN = "/racer/laser/scan";
constexpr const char* TOPIC_LASER_SCAN_POINTCLOUD = "/racer/laser/tf_pointcloud";
constexpr const char* MODEL_BASE_LINK = "base_link";

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
    LaserscanTransformer();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_laserscan_subscriber;
    ros::Publisher m_pointcloud_publisher;
    laser_geometry::LaserProjection m_projector;
    tf::TransformListener m_listener;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
};