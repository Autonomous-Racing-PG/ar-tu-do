#include "wall_following.h"

WallFollowing::WallFollowing()
{
    lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 1, &WallFollowing::lidarCallback, this);
    emer_stop_subscriber =
        m_node_handle.subscribe<std_msgs::Bool>(TOPIC_EMER_STOP, 1, &WallFollowing::emergencyStopCallback, this);
    pid_publisher = m_node_handle.advertise<drive_msgs::pid_input>(TOPIC_PID_INPUT, 1);
}

float WallFollowing::rangeAtDegree(const sensor_msgs::LaserScan::ConstPtr& lidar, float theta)
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

std::array<float, 2> WallFollowing::followRightWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    // calculations done according to the F1Tenth tutorial
    // http://f1tenth.org/lab_instructions/t6.pdf

    // some angle theta between 0 and 70 which you can choose by yourself
    float theta = 95;

    // two distances a and b
    // with b being the distance at angle 0 and a being the distance at angle theta
    // the lidar has a range of 270°, the algorithm use 180°
    // so the the angle 0° for the algorith starts at angle 45° for the lidar
    float a = rangeAtDegree(lidar, theta);
    float b = rangeAtDegree(lidar, 45);

    // transform the difference between the angles at a and b into radians
    float swing = (std::abs(theta - 45) * M_PI) / 180;

    // alpha is the orientation of the car with respect to the wall
    // and a_b is the distance to the wall
    // a_c is the distance between the current position of the car and the predicted position
    // we use the car_length as a_c for now PLEASE CHANGE IN THE FUTURE
    // c_d is the distance to the wall at the predicted position
    float alpha = std::atan((a * std::cos(swing) - b) / (a * std::sin(swing)));
    float a_b = b * std::cos(alpha);
    float a_c = 0.5;
    float c_d = a_b + a_c * std::sin(alpha);

    // values used for PID control, given be ZIEGLER und NICHOLS
    // periodendauer bei kritischer verstärkung = 300ms
    float critic_kp = 800;
    float critic_period = 0.5;
    float kp = critic_kp * 0.6;
    float ki = 2 * kp / 500;
    float kd = kp * 0.5 / 8;
    float dt = 0.025; // 25ms iteration time

    // if we want to stay 0.5 meter away from the wall
    // then the error is 0.5-CD
    m_right_error = 0.5 - c_d;

    m_right_integral += (m_right_error * dt);
    float right_derivative = (m_right_error - m_right_prev_error) / dt;

    // correction according to the ZIEGLER und NICHOLS
    float correction = kp * m_right_error + ki * (m_right_integral) + kd * right_derivative;

    m_right_prev_error = m_right_error;

    // use correction to increment or decrement steering angle
    // IMPORTANT: for the right wall we have to use the negative value
    // in contrast to the positive value for the left wall]
    m_right_corrected_angle = -(correction * M_PI) / 180;

    // check if speed is too high (car cannot react fast enough)
    // or too low/negative (because we substract the corrected angle from the max speed)
    float m_right_velocity = WALL_FOLLOWING_MAX_SPEED - std::abs(m_right_corrected_angle) * WALL_FOLLOWING_MAX_SPEED;

    if (m_right_velocity > WALL_FOLLOWING_MAX_SPEED)
    {
        m_right_velocity = WALL_FOLLOWING_MAX_SPEED;
    }
    if (m_right_velocity < WALL_FOLLOWING_MIN_SPEED)
    {
        m_right_velocity = WALL_FOLLOWING_MIN_SPEED;
    }

    return { std::atan(m_right_corrected_angle), m_right_velocity };
}

std::array<float, 2> WallFollowing::followLeftWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    // calculations done according to the F1Tenth tutorial
    // http://f1tenth.org/lab_instructions/t6.pdf

    // some angle theta between 0 and 70 which you can choose by yourself
    // for the left wall we have to start at the left sid of the lidar
    // which is at angle 225 and subtract the theta (here 50) so we get 175
    float theta = 175;

    // two distances a and b
    // with b being the distance at angle 180° and a being the distance at angle theta
    // the lidar has a range of 270°, the algorithm use 180°
    // so the the angle 180° for the algorith starts at angle 225° for the lidar
    float a = rangeAtDegree(lidar, theta);
    float b = rangeAtDegree(lidar, 225);

    // transform the difference between the angles at a and b into radians
    float swing = (std::abs(theta - 225) * M_PI) / 180;

    // alpha is the orientation of the car with respect to the wall
    // and a_b is the distance to the wall
    // a_c is the distance between the current position of the car and the predicted position
    // we use the car_length as a_c for now PLEASE CHANGE IN THE FUTURE
    // c_d is the distance to the wall at the predicted position
    float alpha = std::atan((a * std::cos(swing) - b) / (a * std::sin(swing)));
    float a_b = b * std::cos(alpha);
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
    m_left_error = 0.5 - c_d;

    m_left_integral += (m_left_error * dt);
    float left_derivative = (m_left_error - m_left_prev_error) / dt;

    // correction according to the ZIEGLER und NICHOLS
    float correction = kp * m_left_error + ki * (m_left_integral) + kd * left_derivative;

    m_left_prev_error = m_left_error;

    // use correction to increment or decrement steering angle
    // IMPORTANT: for the left wall we have to use the positive value
    // in contrast to the negative value for the right wall
    m_left_corrected_angle = (correction * M_PI) / 180;

    // check if speed is too high (car cannot react fast enough)
    // or too low/negative (because we substract the corrected angle from the max speed)
    float m_left_velocity = WALL_FOLLOWING_MAX_SPEED - std::abs(m_left_corrected_angle) * WALL_FOLLOWING_MAX_SPEED;

    if (m_left_velocity > WALL_FOLLOWING_MAX_SPEED)
    {
        m_left_velocity = WALL_FOLLOWING_MAX_SPEED;
    }
    if (m_left_velocity < WALL_FOLLOWING_MIN_SPEED)
    {
        m_left_velocity = WALL_FOLLOWING_MIN_SPEED;
    }

    return { std::atan(m_left_corrected_angle), m_left_velocity };
}
void WallFollowing::emergencyStopCallback(const std_msgs::Bool emer_stop)
{
    m_emergency_stop = emer_stop.data;
}

void WallFollowing::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    if (m_emergency_stop == false)
    {
        auto wall_values = m_follow_right_wall ? followRightWall(lidar) : followLeftWall(lidar);
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