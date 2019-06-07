#include "wall_following.h"
#include <boost/algorithm/clamp.hpp>

WallFollowing::WallFollowing()
    : m_debug_geometry(this->m_node_handle, TOPIC_VISUALIZATION, LIDAR_FRAME)
{
    this->m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallFollowing::lidarCallback, this);
    this->m_drive_parameter_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);

    this->updateDynamicConfig();

    m_dyn_cfg_server.setCallback([&](wallfollowing1::wallfollowing1Config& cfg, uint32_t) {
        m_min_range = cfg.min_range;
        m_max_range = cfg.max_range;

        m_fallback_range = cfg.fallback_range;

        m_sample_angle_1 = static_cast<float>(cfg.sample_angle_1) * DEG_TO_RAD;
        m_sample_angle_2 = static_cast<float>(cfg.sample_angle_2) * DEG_TO_RAD;

        m_wall_following_max_speed = cfg.wall_following_max_speed;
        m_wall_following_min_speed = cfg.wall_following_min_speed;

        m_prediction_distance = cfg.prediction_distance;

        m_target_wall_distance = cfg.target_wall_distance;

        m_time_between_scans = cfg.time_between_scans;
    });
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
        || lidar->ranges[index] < m_min_range
        || lidar->ranges[index] > m_max_range) {
        ROS_INFO_STREAM("Could not sample lidar, using fallback value");
        return m_fallback_range;
    }
    // clang-format on

    return lidar->ranges[index];
}

Wall WallFollowing::getWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall)
{
    float leftRightSign = right_wall ? -1 : 1;

    float angle1 = m_sample_angle_1 * leftRightSign;
    float angle2 = m_sample_angle_2 * leftRightSign;
    float range1 = this->getRangeAtDegree(lidar, angle1);
    float range2 = this->getRangeAtDegree(lidar, angle2);

    return Wall(angle1, angle2, range1, range2);
}

/**
 * @brief This method attempts to follow either the right or left wall.
 * It samples two points next to the car and estimates a straight wall based on them.
 * The predicted distance is the distance between the wall and the position that the car
 * will have after it drives straight forward for m_prediction_distance meters.
 * A PID controller is used to minimize the difference between the predicted distance
 * to the wall and TARGET_DISTANCE.
 * The calculation is based on this document: http://f1tenth.org/lab_instructions/t6.pdf
 */
void WallFollowing::followSingleWall(const sensor_msgs::LaserScan::ConstPtr& lidar, bool right_wall)
{
    float leftRightSign = right_wall ? -1 : 1;

    Wall wall = this->getWall(lidar, right_wall);
    float predictedWallDistance = wall.predictDistance(m_prediction_distance);

    float error = m_target_wall_distance - predictedWallDistance;
    float correction = this->m_pid_controller.updateAndGetCorrection(error, m_time_between_scans);

    float steeringAngle = atan(leftRightSign * correction) * 2 / M_PI;
    float velocity = m_wall_following_max_speed * (1 - std::abs(steeringAngle));
    velocity = boost::algorithm::clamp(velocity, m_wall_following_min_speed, m_wall_following_max_speed);

    wall.draw(this->m_debug_geometry, 0, createColor(0, 0, 1, 1));
    this->m_debug_geometry.drawLine(1, createPoint(m_prediction_distance, 0, 0),
                                    createPoint(m_prediction_distance, -error * leftRightSign, 0),
                                    createColor(1, 0, 0, 1), 0.03);
    float wallAngle = wall.getAngle();
    this->m_debug_geometry.drawLine(2, createPoint(m_prediction_distance, -error * leftRightSign, 0),
                                    createPoint(m_prediction_distance + std::cos(wallAngle) * 2,
                                                (-error + std::sin(wallAngle) * 2) * leftRightSign, 0),
                                    createColor(0, 1, 1, 1), 0.03);

    this->publishDriveParameters(velocity, steeringAngle);
}

void WallFollowing::followWalls(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    Wall leftWall = this->getWall(lidar, false);
    Wall rightWall = this->getWall(lidar, true);

    float error =
        (rightWall.predictDistance(m_prediction_distance) - leftWall.predictDistance(m_prediction_distance)) / 2;
    float correction = this->m_pid_controller.updateAndGetCorrection(error, m_time_between_scans);

    float steeringAngle = atan(correction) * 2 / M_PI;
    float velocity = m_wall_following_max_speed * (1 - std::max(0.0f, std::abs(steeringAngle) - 0.15f));
    velocity = boost::algorithm::clamp(velocity, m_wall_following_min_speed, m_wall_following_max_speed);

    leftWall.draw(this->m_debug_geometry, 0, createColor(0, 0, 1, 1));
    rightWall.draw(this->m_debug_geometry, 1, createColor(0, 0, 1, 1));
    this->m_debug_geometry.drawLine(2, createPoint(m_prediction_distance, 0, 0),
                                    createPoint(m_prediction_distance, -error, 0), createColor(1, 0, 0, 1), 0.03);
    float distance2 = m_prediction_distance + 2;
    this->m_debug_geometry.drawLine(
        3, createPoint(m_prediction_distance, -error, 0),
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

void WallFollowing::updateDynamicConfig()
{
    wallfollowing1::wallfollowing1Config cfg;
    {
        cfg.min_range = m_min_range;
        cfg.max_range = m_max_range;

        cfg.fallback_range = m_fallback_range;

        cfg.sample_angle_1 = m_sample_angle_1 / DEG_TO_RAD;
        cfg.sample_angle_2 = m_sample_angle_2 / DEG_TO_RAD;

        cfg.wall_following_max_speed = m_wall_following_max_speed;
        cfg.wall_following_min_speed = m_wall_following_min_speed;

        cfg.prediction_distance = m_prediction_distance;

        cfg.target_wall_distance = m_target_wall_distance;

        cfg.time_between_scans = m_time_between_scans;
    }
    m_dyn_cfg_server.updateConfig(cfg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_following");
    WallFollowing wall_following;
    ros::spin();
    return EXIT_SUCCESS;
}
