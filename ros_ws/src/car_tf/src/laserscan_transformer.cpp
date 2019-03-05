#include "laserscan_transformer.h"
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

constexpr const char* TOPIC_LASER_SCAN = "/racer/laser/scan";
constexpr const char* TOPIC_LASER_SCAN_POINTCLOUD = "/racer/laser/tf_pointcloud";
constexpr const char* MODEL_BASE_LINK = "base_link";

LaserscanTransformer::LaserscanTransformer()
{
    laserscan_subscriber = m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 100,
                                                                           &LaserscanTransformer::scanCallback, this);
    pointcloud_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(TOPIC_LASER_SCAN_POINTCLOUD, 100, false);
}

void LaserscanTransformer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    sensor_msgs::PointCloud2 pointcloud;
    projector.transformLaserScanToPointCloud(MODEL_BASE_LINK, *laserscan, pointcloud, listener);
    pointcloud_publisher.publish(pointcloud);
}
