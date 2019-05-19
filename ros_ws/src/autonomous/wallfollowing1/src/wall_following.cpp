#include "wall_following.h"
#include <boost/algorithm/clamp.hpp>

WallFollowing::WallFollowing()
    : m_debug_geometry(this->m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallFollowing::lidarCallback, this);
    this->m_drive_parameter_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

// map value in range [in_lower, in_upper] to the corresponding number in range [out_lower, out_upper]
float map(float in_lower, float in_upper, float out_lower, float out_upper, float value)
{
    return out_lower + ((out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower));
}

float WallFollowing::getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle)
{
    int sampleCount = (lidar->angle_max - lidar->angle_min) / lidar->angle_increment;
    int index = map(lidar->angle_min, lidar->angle_max, 0, sampleCount, angle);

    // clang-format off
    if (index < 0
        || index >= sampleCount
        || lidar->ranges[index] < MIN_RANGE
        || lidar->ranges[index] > MAX_RANGE) {
        ROS_INFO_STREAM("Could not sample lidar, using fallback value");
        return FALLBACK_RANGE;
    }
    // clang-format on

    return lidar->ranges[index];
}

Wall WallFollowing::getWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall)
{
    float leftRightSign = right_wall ? -1 : 1;

    float angle1 = SAMPLE_ANGLE_1 * leftRightSign;
    float angle2 = SAMPLE_ANGLE_2 * leftRightSign;
    float range1 = this->getRangeAtDegree(lidar, angle1);
    float range2 = this->getRangeAtDegree(lidar, angle2);

    return Wall(angle1, angle2, range1, range2);
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
void WallFollowing::followSingleWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall)
{
    float leftRightSign = right_wall ? -1 : 1;

    Wall wall = this->getWall(lidar, right_wall);
    float predictedWallDistance = wall.predictDistance(PREDICTION_DISTANCE);

    float error = TARGET_WALL_DISTANCE - predictedWallDistance;
    float correction = this->m_pid_controller.updateAndGetCorrection(error, TIME_BETWEEN_SCANS);

    float steeringAngle = atan(leftRightSign * correction) * 2 / M_PI;
    float velocity = WALL_FOLLOWING_MAX_SPEED * (1 - std::abs(steeringAngle));
    velocity = boost::algorithm::clamp(velocity, WALL_FOLLOWING_MIN_SPEED, WALL_FOLLOWING_MAX_SPEED);

    wall.draw(this->m_debug_geometry, 0, createColor(0, 0, 1, 1));
    this->m_debug_geometry.drawLine(1, createPoint(PREDICTION_DISTANCE, 0, 0),
                                    createPoint(PREDICTION_DISTANCE, -error * leftRightSign, 0),
                                    createColor(1, 0, 0, 1), 0.03);
    float wallAngle = wall.getAngle();
    this->m_debug_geometry.drawLine(2, createPoint(PREDICTION_DISTANCE, -error * leftRightSign, 0),
                                    createPoint(PREDICTION_DISTANCE + std::cos(wallAngle) * 2,
                                                (-error + std::sin(wallAngle) * 2) * leftRightSign, 0),
                                    createColor(0, 1, 1, 1), 0.03);

    this->publishDriveParameters(velocity, steeringAngle);
}

void WallFollowing::followWalls(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    Wall leftWall = this->getWall(lidar, false);
    Wall rightWall = this->getWall(lidar, true);

    float error = (rightWall.predictDistance(PREDICTION_DISTANCE) - leftWall.predictDistance(PREDICTION_DISTANCE)) / 2;
    float correction = this->m_pid_controller.updateAndGetCorrection(error, TIME_BETWEEN_SCANS);

    float steeringAngle = atan(correction) * 2 / M_PI;
    float velocity = WALL_FOLLOWING_MAX_SPEED * (1 - std::max(0.0f, std::abs(steeringAngle) - 0.15f));
    velocity = boost::algorithm::clamp(velocity, WALL_FOLLOWING_MIN_SPEED, WALL_FOLLOWING_MAX_SPEED);

    leftWall.draw(this->m_debug_geometry, 0, createColor(0, 0, 1, 1));
    rightWall.draw(this->m_debug_geometry, 1, createColor(0, 0, 1, 1));
    this->m_debug_geometry.drawLine(2, createPoint(PREDICTION_DISTANCE, 0, 0),
                                    createPoint(PREDICTION_DISTANCE, -error, 0), createColor(1, 0, 0, 1), 0.03);
    float distance2 = PREDICTION_DISTANCE + 2;
    this->m_debug_geometry.drawLine(
        3, createPoint(PREDICTION_DISTANCE, -error, 0),
        createPoint(distance2, -(rightWall.predictDistance(distance2) - leftWall.predictDistance(distance2)) / 2, 0),
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

void WallFollowing::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    this->followWalls(lidar);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_following");
    WallFollowing wall_following;
    ros::spin();
    return EXIT_SUCCESS;
}
