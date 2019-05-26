#include "emergency_stop.h"
#include <std_msgs/Time.h>

EmergencyStop::EmergencyStop()
{
    m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &EmergencyStop::lidarCallback, this);
    m_emergency_stop_publisher = m_node_handle.advertise<std_msgs::Time>(TOPIC_EMERGENCY_STOP, 1);
}

bool EmergencyStop::emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    ROS_ASSERT_MSG(RANGE_THRESHOLD <= MAX_RANGE,
                   "Threshold is bigger then max range. Function will always return false.");
    ROS_ASSERT_MSG(MAX_RANGE > 0, "Max range is zero or below. Function will mostly return true.");
    ROS_ASSERT_MSG(RANGE_THRESHOLD > 0, "Threshold is zero or below. Function will mostly return false.");

    int available_samples = (lidar->angle_max - lidar->angle_min) / lidar->angle_increment;

    // calculate the sample_angle of the lidar to be used to check if there is
    // an obstacle in front of the car, with respect to the RANGE_THRESHOLD
    // We want to check every lidar sample that is located in front of the car at a distance of RANGE_THRESHOLD.
    float sample_angle = std::atan(CAR_BUMPER_LENGTH / 2 / RANGE_THRESHOLD);

    int index_start = available_samples / 2 - sample_angle / lidar->angle_increment / 2;
    int index_end = available_samples / 2 + sample_angle / lidar->angle_increment / 2;
    ROS_ASSERT_MSG(index_start >= 0 && index_end < available_samples,
                   "sample_angle is too big. Trying to access lidar samples out of bounds.");

    // determine the minimum distance between the car and a potetial obstacle
    // Instead of calculating the range average, we are more cautious because we would
    // rather have false positives instead of false negatives.
    // i.e. we would rather stop too much than crash into an obstacle.
    auto min_range =
        std::min(MAX_RANGE, *std::min_element(lidar->ranges.begin() + index_start, lidar->ranges.begin() + index_end));
    ROS_ASSERT_MSG(min_range >= 0, "The minimal distance between the car and a potential obstacle is below zero.");

    return min_range < RANGE_THRESHOLD;
}

void EmergencyStop::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    bool emergency_stop_active = emergencyStop(lidar);

    if (emergency_stop_active)
    {
        ROS_INFO_STREAM("Wall detected. Emergency stop is active.");

        std_msgs::Time message;
        message.data = ros::Time::now();

        m_emergency_stop_publisher.publish(message);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop");
    EmergencyStop emergency_stop;
    ros::spin();
    return EXIT_SUCCESS;
}