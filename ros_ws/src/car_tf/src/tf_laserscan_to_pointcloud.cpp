#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class LaserscanTransformer {
     public:
        LaserscanTransformer();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node;
        laser_geometry::LaserProjection projector;
        tf::TransformListener listener;
        ros::Publisher point_cloud_publisher;
        ros::Subscriber scan_subscriber;
};

LaserscanTransformer::LaserscanTransformer(){
        scan_subscriber = node.subscribe<sensor_msgs::LaserScan> ("/racer/laser/scan", 100, &LaserscanTransformer::scanCallback, this);
        point_cloud_publisher = node.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
}

void LaserscanTransformer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud("base_link", *scan, cloud, listener);
    point_cloud_publisher.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_laserscan_to_pointcloud");
    LaserscanTransformer tf_laserscan_to_pointcloud;
    ros::spin();

    return 0;
}