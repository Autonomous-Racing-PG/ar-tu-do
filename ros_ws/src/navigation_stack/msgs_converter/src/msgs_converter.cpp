#include "msgs_converter.h"
#include "car_config.h"
#include <Eigen/Dense>

using namespace Eigen;

MSGSConverter::MSGSConverter()
{
    this->m_command_velocity_subscriber =
        this->m_node_handle.subscribe<geometry_msgs::Twist>(car_config::CMD_VEL, 1, &MSGSConverter::convertCallback,
                                                            this);

    this->m_to_car_control_publisher =
        this->m_node_handle.advertise<drive_msgs::drive_param>(car_config::TOPIC_DRIVE_PARAM, 10);
}

void MSGSConverter::convertCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_message)
{

    double velocity_x = cmd_vel_message->linear.x;
    double velocity_y = cmd_vel_message->linear.y;
    double velocity_angular = cmd_vel_message->angular.z;

    Vector2d metric_velocity(velocity_x, velocity_y);

    double velocity_result =
        metric_velocity.norm(); // check if norm is zero, then the steering angle (angle_rad) will be infinity!
    double erpm_speed = metric_velocity.norm() * car_config::TRANSMISSION / car_config::ERPM_TO_SPEED;
    double angle_rad = 0;

    if (velocity_angular != 0)
    {
        if (velocity_result < 0 && velocity_result > -0.001)
        {
            velocity_result = -0.001;
        }
        else if (velocity_result > 0 && velocity_result < 0.001)
        {
            velocity_result = -0.001;
        }

        angle_rad = atan((car_config::WHEELBASE * velocity_angular) / metric_velocity.norm());
    }

    double servo_data = (angle_rad * car_config::STEERING_TO_SERVO_GAIN) + car_config::STEERING_TO_SERVO_OFFSET;
    if (servo_data < 0)
    {
        servo_data = 0;
    }
    else if (servo_data > 1)
    {
        servo_data = 1;
    }

    drive_msgs::drive_param control;
    control.velocity = erpm_speed / car_config::MAX_RPM_ELECTRICAL; // convert from min-max erpm to (-1)-1
    control.angle = (servo_data * 2 - car_config::MAX_SERVO_POSITION) /
        car_config::MAX_SERVO_POSITION; // convert from 0-1 to (-1)-1
    m_to_car_control_publisher.publish(control);
}
