#include "emergency_stop.h"

EmergencyStop::EmergencyStop()
{
    lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &EmergencyStop::lidarCallback, this);
    emer_stop_publisher = m_node_handle.advertise<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1);
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
    // calculate the radius of the lidar to be used to check if there is
    // an obstacle in front of the car, with respect to the EMERGENCY_STOP_DISTANCE
    float radius = (360 * CAR_BUMPER_LENGTH) / (M_PI * EMERGENCY_STOP_DISTANCE * 2);

    // compute the number of lidar samples needed to cover the radius calculated above.
    int lidar_samples = (int) radius / 0.375;

    // determine the minimum distance between the car and a potetial obstacle 
    float min_dist = 100;
    for (int i = 135 - lidar_samples/2 ; i < 135 + lidar_samples/2; i++)
    {
        if(min_dist > rangeAtDegree(lidar, i))
            min_dist = rangeAtDegree(lidar,i);
    }

    // return 0 (stop) if the object is too close
    return (min_dist < EMERGENCY_STOP_DISTANCE);
}

void EmergencyStop::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    bool emergency_stop = emergencyStop(lidar);

    if (emergency_stop)
    {
        ROS_INFO_STREAM("Too close! Stop!");
    }

    std_msgs::Bool emer_stop;
    {
        emer_stop.data = emergency_stop;
    }
    emer_stop_publisher.publish(emer_stop);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop");
    EmergencyStop emergency_stop;
    ros::spin();
    return EXIT_SUCCESS;
}