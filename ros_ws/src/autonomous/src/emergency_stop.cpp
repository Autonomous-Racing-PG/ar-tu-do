#include "emergency_stop.h"

EmergencyStop::EmergencyStop()
{
    m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &EmergencyStop::lidarCallback, this);
    m_emergency_stop_publisher = m_node_handle.advertise<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1);
}

bool EmergencyStop::emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    int available_samples = (lidar->angle_max - lidar->angle_min) / lidar->angle_increment;

    // calculate the sample_angle of the lidar to be used to check if there is
    // an obstacle in front of the car, with respect to the RANGE_THRESHOLD
    float sample_angle = (360 * CAR_BUMPER_LENGTH) / (M_PI * RANGE_THRESHOLD * 2) * DEG_TO_RAD;

    int index_start = available_samples / 2 - sample_angle / lidar->angle_increment / 2;
    int index_end = available_samples / 2 + sample_angle / lidar->angle_increment / 2;
    ROS_ASSERT_MSG(index_start >= 0 && index_end < available_samples,
                   "sample_angle is too big. Trying to access lidar samples out of bounds.");  

    // determine the minimum distance between the car and a potetial obstacle
    // Instead of calculating the range average, we are more cautious because we would 
    // rather have false positives instead of false negatives.
    // i.e. we would rather stop too much than crash into an obstacle.
    float min_dist = MAX_RANGE;
    for (int i = index_start; i < index_end; i++)
    {
        float range = lidar->ranges[i];

        if(min_dist > range)
        {
            min_dist = range;
        }
    }

    return min_dist < RANGE_THRESHOLD;
}

void EmergencyStop::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    bool emergency_stop_active = emergencyStop(lidar);

    if (emergency_stop_active)
    {
        ROS_INFO_STREAM("Wall detected. Emergency stop is active.");
    }

    std_msgs::Bool message;
    {
        message.data = emergency_stop_active;
    }
    m_emergency_stop_publisher.publish(message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop");
    EmergencyStop emergency_stop;
    ros::spin();
    return EXIT_SUCCESS;
}