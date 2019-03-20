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
    int samples_used = 0;
    float range_sum = 0;

    int index_start = available_samples / 2 - SAMPLE_ANGLE / lidar->angle_increment / 2;
    int index_end = available_samples / 2 + SAMPLE_ANGLE / lidar->angle_increment / 2;
    ROS_ASSERT_MSG(index_start >= 0 && index_end < available_samples,
                   "SAMPLE_ANGLE is too big. Trying to access lidar samples out of bounds.");

    for (int i = index_start; i < index_end; i++)
    {
        float range = lidar->ranges[i];

        if (range < MAX_RANGE && !std::isinf(range))
        {
            range_sum += range;
            samples_used++;
        }
    }

    if (samples_used == 0)
    {
        return false;
    }

    float range_average = range_sum / samples_used;
    return range_average < RANGE_THRESHOLD;
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