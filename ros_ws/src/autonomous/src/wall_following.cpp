#include "wall_following.h"

WallFollowing::WallFollowing()
{
    lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallFollowing::lidarCallback, this);
    emer_stop_subscriber =
        m_node_handle.subscribe<std_msgs::Bool>(TOPIC_EMERGENCY_STOP, 1, &WallFollowing::emergencyStopCallback, this);
    pid_publisher = m_node_handle.advertise<drive_msgs::pid_input>(TOPIC_PID_INPUT, 1);
}

float map(float in_lower, float in_upper, float out_lower, float out_upper, float value)
{
    return out_lower + (out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower);
}

bool WallFollowing::getRangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float angle, float& range)
{
    float angleRad = angle * DEG_TO_RAD;
    int index = map(lidar->angle_min, lidar->angle_max, 0, LIDAR_SAMPLE_COUNT, angleRad);

    if (index < 0 || index >= LIDAR_SAMPLE_COUNT) {
        return false;
    }

    float result = lidar->ranges[index];

    if (result < MIN_RANGE || result > MAX_RANGE) {
        return false;
    }

    range = result;
    return true;
}

std::array<float, 2> WallFollowing::followWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    float leftRightSign = this->m_follow_right_wall ? -1 : 1;

    float range1 = DEFAULT_RANGE;
    float range2 = DEFAULT_RANGE;
    
    if (!this->getRangeAtDegree(lidar, SAMPLE_ANGLE_1 * leftRightSign, range1)) {
        ROS_INFO_STREAM("Could not sample lidar, using default value");
    };
    if (!this->getRangeAtDegree(lidar, SAMPLE_ANGLE_2 * leftRightSign, range2)) {
        ROS_INFO_STREAM("Could not sample lidar, using default value");
    }

    // These calculations are based on this document: http://f1tenth.org/lab_instructions/t6.pdf

    float wallAngle = std::atan((range1 * std::cos(SAMPLE_WINDOW_SIZE * DEG_TO_RAD) - range2) / (range1 * std::sin(SAMPLE_WINDOW_SIZE * DEG_TO_RAD)));
    float currentWallDistance = range2 * std::cos(wallAngle);
    float predictedWallDistance = currentWallDistance + PREDICTION_DISTANCE * std::sin(wallAngle);
    
    float error = TARGET_WALL_DISTANCE - predictedWallDistance;

    float kp = 120;
    float ki = 0.48;
    float kd = 7.5;
    float dt = 0.025; // 25ms iteration time
    
    this->m_integral += error * dt;
    float derivative = (error - m_prev_error) / dt;

    float correction = kp * error + ki * this->m_integral + kd * derivative;

    this->m_prev_error = error;

    float correctedAngle = leftRightSign * correction * DEG_TO_RAD;

    // check if speed is too high (car cannot react fast enough)
    // or too low/negative (because we substract the corrected angle from the max speed)
    float m_velocity = WALL_FOLLOWING_MAX_SPEED - std::abs(correctedAngle) * WALL_FOLLOWING_MAX_SPEED;



    if (m_velocity > WALL_FOLLOWING_MAX_SPEED)
    {
        m_velocity = WALL_FOLLOWING_MAX_SPEED;
    }
    if (m_velocity < WALL_FOLLOWING_MIN_SPEED)
    {
        m_velocity = WALL_FOLLOWING_MIN_SPEED;
    }

    return { std::atan(correctedAngle), m_velocity };
}

void WallFollowing::emergencyStopCallback(const std_msgs::Bool emer_stop)
{
    m_emergency_stop = emer_stop.data;
}

void WallFollowing::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    if (m_emergency_stop == false)
    {
        auto wall_values = this->followWall(lidar);
        auto corrected_angle = wall_values[0];
        auto velocity = wall_values[1];

        drive_msgs::pid_input corr;
        corr.pid_error = corrected_angle;
        corr.pid_vel = velocity;
        pid_publisher.publish(corr);
    }
    else
    {
        drive_msgs::pid_input emer_stop;
        emer_stop.pid_vel = 0;
        emer_stop.pid_error = 0;
        pid_publisher.publish(emer_stop);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_following");
    WallFollowing wall_following;
    ros::spin();
    return EXIT_SUCCESS;
}