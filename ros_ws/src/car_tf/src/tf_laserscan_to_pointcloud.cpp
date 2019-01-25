#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class LaserscanTransformer {
     public:
        LaserscanTransformer();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan);
     private:
        ros::NodeHandle node;
        laser_geometry::LaserProjection projector;
        tf::TransformListener listener;
        ros::Publisher pointcloud_publisher;
        ros::Subscriber laserscan_subscriber;
};

LaserscanTransformer::LaserscanTransformer(){
        laserscan_subscriber = node.subscribe<sensor_msgs::LaserScan> ("/racer/laser/scan", 100, &LaserscanTransformer::scanCallback, this);
        pointcloud_publisher = node.advertise<sensor_msgs::PointCloud2> ("/racer/laser/tf_pointcloud", 100, false);
}

void LaserscanTransformer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan){
    sensor_msgs::PointCloud2 pointcloud;
    projector.transformLaserScanToPointCloud("base_link", *laserscan, pointcloud, listener);
    pointcloud_publisher.publish(pointcloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_laserscan_to_pointcloud");
    LaserscanTransformer tf_laserscan_to_pointcloud;
    ros::spin();

    return 0;
}