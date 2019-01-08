#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "drive_msgs/pid_input.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include <ros/console.h>
#include <ros/ros.h>

float g_right_error = 0;
float g_right_prev_error = 0;
float g_right_corrected_angle = 0;

float g_left_error = 0;
float g_left_prev_error = 0;
float g_left_corrected_angle = 0;

class WallFollowing
{
    public:
    WallFollowing();

    private:
    ros::NodeHandle m_node_handle;

    ros::Subscriber lidar_subscriber;
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar);

    ros::Publisher pid_publisher;

    // void adjustSpeed(double speed);
};

WallFollowing::WallFollowing()
{
    lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>("/racer/laser/scan", 1, &WallFollowing::lidarCallback, this);
    pid_publisher = m_node_handle.advertise<drive_msgs::pid_input>("/pid_input", 1);
}

/**
 * @brief Using the lidar scans and a given angle theta
 * this method returns the range at the given theta after doing boundary checks.
 *
 * @param lidar
 * @param theta
 * @return float
 */
float getRange(const sensor_msgs::LaserScan::ConstPtr& lidar, float theta)
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

/**
 * @brief Checks if there is an unavoidable wall in front of the car using the lidar scans
 * and returns true to inhibit driving if it is the case.
 *
 * @param lidar
 * @return bool
 */
bool emergencyStop(const sensor_msgs::LaserScan::ConstPtr& lidar)
{

    // used for range averaging
    float front_range_sum = 0;

    // calculate the average distance in front of the car (6° radius)
    // used for robustness
    // (e.g. there is noise and only a single index shows a close range...
    // ... this is probably not an obstacle)
    for (int i = 133; i < 139; i++)
    {
        front_range_sum += getRange(lidar, i);
        // ROS_INFO_STREAM("front range sum: " << front_range_sum);
    }

    // return 0 (stop) if the object is too close
    if ((front_range_sum / 6) < 0.3)
    {
        ROS_INFO_STREAM("too close!: " << front_range_sum);
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Using the lidar scans this method calculates the necesarry angle and velocity
 * for the car to stay parallel to the right wall and then returns them both in that order.
 *
 * @param lidar
 * @return std::array<float, 2>
 */

std::array<float, 2> followRightWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    // calculations done according to the F1Tenth tutorial
    // http://f1tenth.org/lab_instructions/t6.pdf

    // some angle theta between 0 and 70 which you can choose by yourself
    float theta = 95;

    // two distances a and b
    // with b being the distance at angle 0 and a being the distance at angle theta
    // the lidar has a range of 270°, the algorithm use 180°
    // so the the angle 0° for the algorith starts at angle 45° for the lidar
    float a = getRange(lidar, theta);
    float b = getRange(lidar, 45);

    ROS_INFO_STREAM("right a: " << a);
    ROS_INFO_STREAM("right b: " << b);

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

    ROS_INFO_STREAM("right_alpha: " << alpha);
    ROS_INFO_STREAM("right_a_b: " << a_b);
    ROS_INFO_STREAM("right_c_d: " << c_d);

    // values used for PID control, given be the F1Tenth tutorial
    float kp = 14;
    float kd = 0.09;

    // if we want to stay 0.5 meter away from the wall
    // then the error is 0.5-CD
    g_right_error = 0.5 - c_d;

    ROS_INFO_STREAM("right error: " << g_right_error);

    // scale error so the car can react fast enough
    g_right_error = 5 * g_right_error;

    ROS_INFO_STREAM("scaled_right error: " << g_right_error);

    // correction according to the F1Tenth tutorial
    float correction = kp * g_right_error + kd * (g_right_prev_error - g_right_error);

    // std::cout << "correction:" << correction << std::endl;
    ROS_INFO_STREAM("right correction: " << correction);

    g_right_prev_error = g_right_error;

    // use correction to increment or decrement steering angle
    g_right_corrected_angle = (correction * M_PI) / 180;

    // check if speed is too high (car cannot react fast enough)
    // or too low/negative (because we substract the corrected angle from the max speed)
    float g_right_velocity = 4 - std::abs(g_right_corrected_angle);

    if (g_right_velocity > 4)
    {
        g_right_velocity = 4;
    }
    if (g_right_velocity < 0.3)
    {
        g_right_velocity = 0.3;
    }

    return { g_right_corrected_angle, g_right_velocity };
}

/**
 * @brief Using the lidar scans this method calculates the necesarry angle and velocity
 * for the car to stay parallel to the left wall and then returns them both in that order.
 *
 * @param lidar
 * @return std::array<float, 2>
 */
std::array<float, 2> followLeftWall(const sensor_msgs::LaserScan::ConstPtr& lidar)
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
    float a = getRange(lidar, theta);
    float b = getRange(lidar, 225);

    ROS_INFO_STREAM("left a: " << a);
    ROS_INFO_STREAM("left b: " << b);

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

    ROS_INFO_STREAM("left_alpha: " << alpha);
    ROS_INFO_STREAM("left_a_b: " << a_b);
    ROS_INFO_STREAM("left_c_d: " << c_d);

    // values used for PID control, given be the F1Tenth tutorial
    float kp = 14;
    float kd = 0.09;

    // if we want to stay 0.5 meter away from the wall
    // then the error is 0.5-CD
    g_left_error = 0.5 - c_d;

    ROS_INFO_STREAM("left error: " << g_left_error);

    // scale error so the car can react fast enough
    g_left_error = 5 * g_left_error;

    ROS_INFO_STREAM("scaled left error: " << g_left_error);

    // correction according to the F1Tenth tutorial
    float correction = kp * g_left_error + kd * (g_left_prev_error - g_left_error);

    ROS_INFO_STREAM("left correction: " << correction);

    g_left_prev_error = g_left_error;

    // use correction to increment or decrement steering angle
    // IMPORTANT: for the left wall we have to use the negative value
    // in contrast to the positive value for the right wall
    g_left_corrected_angle = -(correction * M_PI) / 180;

    // check if speed is too high (car cannot react fast enough)
    // or too low/negative (because we substract the corrected angle from the max speed)
    float g_left_velocity = 4 - std::abs(g_left_corrected_angle);

    if (g_left_velocity > 4)
    {
        g_left_velocity = 4;
    }
    if (g_left_velocity < 0.3)
    {
        g_left_velocity = 0.3;
    }

    return { g_left_corrected_angle, g_left_velocity };
}

/**
 * @brief The callback method for this node. Gets the lidar input and handles the autonomous
 * controlloling and emergency stop.
 *
 * @param lidar
 */
void WallFollowing::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{

    float emergency_stop = emergencyStop(lidar);
    // ROS_INFO_STREAM("emergency stop: " << emergency_stop);

    if (emergency_stop == false)
    {
        auto right_wall_values = followRightWall(lidar);
        auto right_corrected_angle = right_wall_values[0];
        auto right_velocity = right_wall_values[1];

        auto left_wall_values = followLeftWall(lidar);
        auto left_corrected_angle = left_wall_values[0];
        auto left_velocity = left_wall_values[1];

        ROS_INFO_STREAM("==========================");
        ROS_INFO_STREAM("right corrected angle: " << right_corrected_angle);
        ROS_INFO_STREAM("left corrected angle: " << left_corrected_angle);
        ROS_INFO_STREAM(std::endl);

        drive_msgs::pid_input corr;
        corr.pid_error = left_corrected_angle;
        corr.pid_vel = left_velocity;
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