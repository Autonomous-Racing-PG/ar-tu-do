#include "navigation_stack_control_converter.h"
#include "car_config.h"
#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

NavigationStackControlConverter::NavigationStackControlConverter()
{
    this->m_command_velocity_subscriber =
        this->m_node_handle.subscribe<geometry_msgs::Twist>(TOPIC_CMD_VEL, 1,
                                                            &NavigationStackControlConverter::convertCallback, this);

    this->m_drive_param_publisher = this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 10);
}

void NavigationStackControlConverter::convertCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_message)
{

    double velocity = cmd_vel_message->linear.x;
    double angle = cmd_vel_message->angular.z;

    double erpm_speed = velocity * car_config::TRANSMISSION / car_config::ERPM_TO_SPEED;
    double angle_rad = 0;

    /*
    if (std::abs(velocity_angular) < ANGULAR_VELOCITY_THRESHOLD)
    {
        if (velocity_result < 0 && velocity_result > -VELOCITY_THRESHOLD)
        {
            velocity_result = -VELOCITY_THRESHOLD;
        }
        else if (velocity_result > 0 && velocity_result < VELOCITY_THRESHOLD)
        {
            velocity_result = VELOCITY_THRESHOLD;
        }
    }
    */

    double servo_data = (angle * car_config::STEERING_TO_SERVO_GAIN) + car_config::STEERING_TO_SERVO_OFFSET;

    servo_data = boost::algorithm::clamp(servo_data, 0.0, 1.0);

    drive_msgs::drive_param control_message;
    control_message.velocity = erpm_speed / car_config::MAX_RPM_ELECTRICAL; // convert from min-max erpm to (-1)-1
    control_message.angle = (servo_data * 2 - car_config::MAX_SERVO_POSITION) /
        car_config::MAX_SERVO_POSITION; // convert from 0-1 to (-1)-1
    m_drive_param_publisher.publish(control_message);
}
