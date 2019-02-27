#include "emergency_stop.h"

EmergencyStop::EmergencyStop()
{
    lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &EmergencyStop::lidarCallback, this);
    emer_stop_publisher = m_node_handle.advertise<std_msgs::Bool>(TOPIC_EMER_STOP, 1);
}

float EmergencyStop::rangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float theta)
{
    // check if calculated index is invalid
    // if so then return the chosen max range (here 4)
    if (theta / 0.375f < 0 || theta / 0.375f > 719)
    {
        return 4;
    }

    // ranges size is 720, angle range is 270°
    // so one step equals an angle of 0,375°
    // to access range at angle theta, you have to divide the given theta by the step size
    float range = lidar->ranges[theta / 0.375];

    // check for absurd or nan values
    // the max range should be 4 so the car doesn't get affected by walls very far away
    if (range > 4 || std::isinf(range))
    {
        range = 4;
    }
    return range;
}

bool EmergencyStop::emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar)
{

    // used for range averaging
    float front_range_sum = 0;

    // calculate the average distance in front of the car (6° radius)
    // used for robustness
    // (e.g. there is noise and only a single index shows a close range...
    // ... this is probably not an obstacle)
    for (int i = 133; i < 139; i++)
    {
        front_range_sum += rangeAtDegree(lidar, i);
    }

    // return 0 (stop) if the object is too close
    if ((front_range_sum / 6) < 0.3)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void EmergencyStop::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{

    bool emergency_stop = emergencyStop(lidar);

    if (emergency_stop == false)
    {
        ROS_INFO_STREAM("Okay, go on");
        std_msgs::Bool emer_stop;
        emer_stop.data = false;
        emer_stop_publisher.publish(emer_stop);
    }
    else
    {
        ROS_INFO_STREAM("Too close! Stop!");
        std_msgs::Bool emer_stop;
        emer_stop.data = true;
        emer_stop_publisher.publish(emer_stop);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop");
    EmergencyStop emergency_stop;
    ros::spin();
    return EXIT_SUCCESS;
}