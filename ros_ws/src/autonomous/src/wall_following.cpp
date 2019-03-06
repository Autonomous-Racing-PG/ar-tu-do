#include "wall_following.h"
#include <boost/algorithm/clamp.hpp>

WallFollowing::WallFollowing()
{
    this->m_lidar_subscriber =
        this->m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallFollowing::lidarCallback, this);
    this->m_emergency_stop_subscriber =
        this->m_node_handle.subscribe<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1, &WallFollowing::emergencyStopCallback, this);
    this->m_drive_parameter_publisher = this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

float map(float in_lower, float in_upper, float out_lower, float out_upper, float value)
{
    return out_lower + (out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower);
}

bool WallFollowing::getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle, float& range)
{
    int index = map(lidar->angle_min, lidar->angle_max, 0, LIDAR_SAMPLE_COUNT, angle * DEG_TO_RAD);

    // clang-format off
    if (index < 0
        || index >= LIDAR_SAMPLE_COUNT
        || lidar->ranges[index] < MIN_RANGE
        || lidar->ranges[index] > MAX_RANGE) {
        ROS_INFO_STREAM("Could not sample lidar, using fallback value");
        return FALLBACK_RANGE;
    }
    // clang-format on

    return lidar->ranges[index];
}

void WallFollowing::followWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    float leftRightSign = this->m_follow_right_wall ? -1 : 1;

    float range1 = this->getRangeAtDegree(lidar, SAMPLE_ANGLE_1 * leftRightSign);
    float range2 = this->getRangeAtDegree(lidar, SAMPLE_ANGLE_2 * leftRightSign);

    // These calculations are based on this document: http://f1tenth.org/lab_instructions/t6.pdf

    float wallAngle = std::atan((range1 * std::cos(SAMPLE_WINDOW_SIZE * DEG_TO_RAD) - range2) / (range1 * std::sin(SAMPLE_WINDOW_SIZE * DEG_TO_RAD)));
    float currentWallDistance = range2 * std::cos(wallAngle);
    float predictedWallDistance = currentWallDistance + PREDICTION_DISTANCE * std::sin(wallAngle);

    float error = TARGET_WALL_DISTANCE - predictedWallDistance;
    float correction = this->m_pid_controller.updateAndGetCorrection(error, DELTA_TIME);    
    
    float steeringAngle = std::atan(leftRightSign * correction * DEG_TO_RAD);
    float velocity = WALL_FOLLOWING_MAX_SPEED * (1 - std::abs(steeringAngle));
    velocity = boost::algorithm::clamp(velocity, WALL_FOLLOWING_MIN_SPEED, WALL_FOLLOWING_MAX_SPEED);
    
    this->publishDriveParameters(velocity, steeringAngle);
}

void WallFollowing::publishDriveParameters(float velocity, float angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = angle;
    this->m_drive_parameter_publisher.publish(drive_parameters);
}

void WallFollowing::emergencyStopCallback(const std_msgs::Bool emergency_stop_message)
{
    m_emergency_stop = emergency_stop_message.data;
}

void WallFollowing::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    if (m_emergency_stop == false)
    {
        this->followWall(lidar);
    }
    else
    {
        this->publishDriveParameters(0, 0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_following");
    WallFollowing wall_following;
    ros::spin();
    return EXIT_SUCCESS;
}