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

    // alpha is the orientation of the car with respect to the wall
    // and a_b is the distance to the wall
    // a_c is the distance between the current position of the car and the predicted position
    // we use the car_length as a_c for now PLEASE CHANGE IN THE FUTURE
    // c_d is the distance to the wall at the predicted position
    float alpha = std::atan((range1 * std::cos(SAMPLE_WINDOW_SIZE * DEG_TO_RAD) - range2) / (range1 * std::sin(SAMPLE_WINDOW_SIZE * DEG_TO_RAD)));
    float a_b = range2 * std::cos(alpha);
    float a_c = 0.5;
    float c_d = a_b + a_c * std::sin(alpha);
    
    // values used for PID control, given be ZIEGLER und NICHOLS
    // periodendauer bei kritischer verstärkung = 300ms
    float critic_kp = 200;
    float critic_period = 0.5;
    float kp = critic_kp * 0.6;
    float ki = 2 * kp / 500;
    float kd = kp * 0.5 / 8;
    float dt = 0.025; // 25ms iteration time

    // if we want to stay 0.5 meter away from the wall
    // then the error is 0.5-CD
    m_error = 0.5 - c_d;

    m_integral += (m_error * dt);
    float left_derivative = (m_error - m_prev_error) / dt;

    // correction according to the ZIEGLER und NICHOLS
    float correction = kp * m_error + ki * (m_integral) + kd * left_derivative;

    m_prev_error = m_error;

    // use correction to increment or decrement steering angle
    m_corrected_angle = leftRightSign * correction * DEG_TO_RAD;

    // check if speed is too high (car cannot react fast enough)
    // or too low/negative (because we substract the corrected angle from the max speed)
    float m_velocity = WALL_FOLLOWING_MAX_SPEED - std::abs(m_corrected_angle) * WALL_FOLLOWING_MAX_SPEED;

    if (m_velocity > WALL_FOLLOWING_MAX_SPEED)
    {
        m_velocity = WALL_FOLLOWING_MAX_SPEED;
    }
    if (m_velocity < WALL_FOLLOWING_MIN_SPEED)
    {
        m_velocity = WALL_FOLLOWING_MIN_SPEED;
    }

    return { std::atan(m_corrected_angle), m_velocity };
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