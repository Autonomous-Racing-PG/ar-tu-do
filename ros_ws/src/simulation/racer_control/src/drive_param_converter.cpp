#include "drive_param_converter.h"
#include <cmath>

/**
 * @brief Distance between front and rear axis
 *
 */
constexpr double AXIS_DISTANCE { 32.5 };

/**
 * @brief Distance between center of rear wheels
 *
 */
constexpr double REAR_WHEEL_DISTANCE { 23.3 };

/**
 * @brief Constructor creates subscriber and publisher
 */
DriveParamConverter::DriveParamConverter()
{
    this->m_drive_parameters_subscriber = this->m_node_handle.subscribe<drive_msgs::drive_param>(
        "/set/drive_param", 1, &DriveParamConverter::convertDriveParametersCallback, this);
    this->m_left_rear_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_wheel_back_velocity_controller/command", 1);
    this->m_right_rear_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_wheel_back_velocity_controller/command", 1);
    this->m_left_front_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_wheel_front_velocity_controller/command", 1);
    this->m_right_front_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_wheel_front_velocity_controller/command", 1);

    this->m_left_steering_position_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_steering_hinge_position_controller/command", 1);
    this->m_right_steering_position_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_steering_hinge_position_controller/command", 1);
}

/**
 * @brief Callback for ROS Subscriber
 *
 * @param parameters contains steering angle for center of front axis and vehicle velocity
 */
void DriveParamConverter::convertDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    std_msgs::Float64 throttle;
    throttle.data = static_cast<double>(parameters->velocity) / 0.1;

    this->m_left_rear_wheel_velocity_publisher.publish(throttle);
    this->m_right_rear_wheel_velocity_publisher.publish(throttle);
    this->m_left_front_wheel_velocity_publisher.publish(throttle);
    this->m_right_front_wheel_velocity_publisher.publish(throttle);

    //--------------------------------------------------------------
    AckermannSteeringAngles angles = calculateSteeringAngles(parameters->angle);

    std_msgs::Float64 left_wheel;
    left_wheel.data = angles.left_wheel_angle;

    std_msgs::Float64 right_wheel;
    right_wheel.data = angles.right_wheel_angle;

    this->m_left_steering_position_publisher.publish(left_wheel);
    this->m_right_steering_position_publisher.publish(right_wheel);
}

/**
 * @brief Calculates the angles of the front wheels based on the angle of the center of the front axis with ackerman
 * equation/trigonometry
 *
 * @param angle Angle of the center of the front axis
 * @return Angles One Ackermann angle for each front wheel
 */
DriveParamConverter::AckermannSteeringAngles DriveParamConverter::calculateSteeringAngles(const double& angle)
{
    AckermannSteeringAngles angles;
    double radius = tan(angle + M_PI / 2) * AXIS_DISTANCE;
    angles.left_wheel_angle = -atan(AXIS_DISTANCE / (radius + REAR_WHEEL_DISTANCE / 2));  // left wheel
    angles.right_wheel_angle = -atan(AXIS_DISTANCE / (radius - REAR_WHEEL_DISTANCE / 2)); // right wheel
    return angles;
}