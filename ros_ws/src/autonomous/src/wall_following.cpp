#include "wall_following.h"
#include <boost/algorithm/clamp.hpp>

WallFollowing::WallFollowing()
    : m_debug_geometry(this->m_node_handle, TOPIC_VISUALIZATION, "hokuyo")
{
    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallFollowing::lidarCallback, this);
    this->m_emergency_stop_subscriber =
        m_node_handle.subscribe<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1, &WallFollowing::emergencyStopCallback, this);
    this->m_drive_parameter_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

// map value in range [in_lower, in_upper] to the corresponding number in range [out_lower, out_upper]
float map(float in_lower, float in_upper, float out_lower, float out_upper, float value)
{
    return out_lower + ((out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower));
}

float WallFollowing::getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle)
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

/**
 * @brief This method attempts to follow either the right or left wall.
 * It samples two points next to the car and estimates a straight wall based on them.
 * The predicted distance is the distance between the wall and the position that the car
 * will have after it drives straight forward for PREDICTION_DISTANCE meters.
 * A PID controller is used to minimize the difference between the predicted distance
 * to the wall and TARGET_DISTANCE.
 * The calculation is based on this document: http://f1tenth.org/lab_instructions/t6.pdf
 */
void WallFollowing::followWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    float leftRightSign = this->m_follow_right_wall ? -1 : 1;

    float range1 = this->getRangeAtDegree(lidar, SAMPLE_ANGLE_1 * leftRightSign);
    float range2 = this->getRangeAtDegree(lidar, SAMPLE_ANGLE_2 * leftRightSign);

    float wallAngle = atan((range1 * cos(SAMPLE_WINDOW_SIZE * DEG_TO_RAD) - range2) /
                           (range1 * sin(SAMPLE_WINDOW_SIZE * DEG_TO_RAD)));
    float currentWallDistance = range2 * cos(wallAngle);
    float predictedWallDistance = currentWallDistance + PREDICTION_DISTANCE * sin(wallAngle);

    float error = TARGET_WALL_DISTANCE - predictedWallDistance;
    float correction = this->m_pid_controller.updateAndGetCorrection(error, TIME_BETWEEN_SCANS);

    float steeringAngle = atan(leftRightSign * correction * DEG_TO_RAD);
    float velocity = WALL_FOLLOWING_MAX_SPEED * (1 - std::abs(steeringAngle));
    velocity = boost::algorithm::clamp(velocity, WALL_FOLLOWING_MIN_SPEED, WALL_FOLLOWING_MAX_SPEED);

    this->m_debug_geometry.drawLine(0, createPoint(range1 * cos(SAMPLE_ANGLE_1 * leftRightSign * DEG_TO_RAD),
                                                   range1 * sin(SAMPLE_ANGLE_1 * leftRightSign * DEG_TO_RAD), 0.0),
                                    createPoint(range2 * cos(SAMPLE_ANGLE_2 * leftRightSign * DEG_TO_RAD),
                                                range2 * sin(SAMPLE_ANGLE_2 * leftRightSign * DEG_TO_RAD), 0.0),
                                    createColor(0, 1, 0, 1), 0.03);
    this->m_debug_geometry.drawLine(1, createPoint(PREDICTION_DISTANCE, 0, 0),
                                    createPoint(PREDICTION_DISTANCE, -error, 0), createColor(1, 0, 0, 1), 0.03);
    this->m_debug_geometry.drawLine(2, createPoint(PREDICTION_DISTANCE, -error, 0),
                                    createPoint(PREDICTION_DISTANCE + cos(wallAngle) * 2, -error + sin(wallAngle) * 2,
                                                0),
                                    createColor(0, 1, 1, 1), 0.03);

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
    this->m_emergency_stop = emergency_stop_message.data;
}

void WallFollowing::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    if (this->m_emergency_stop == false)
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
