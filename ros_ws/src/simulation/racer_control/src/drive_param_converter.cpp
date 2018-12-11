#include "drive_param_converter.h"
#include <cmath>

/**
 * @brief Construct a new Drive Param Converter:: Drive Param Converter object
 * 
 */
DriveParamConverter::DriveParamConverter(): m_length(32.5), m_width(23.3)
{
    this->m_drive_parameters_subscriber = this->m_node_handle.subscribe<drive_msgs::drive_param>("/set/drive_param",1,&DriveParamConverter::convertDriveParametersCallback,this);
    this->m_left_rear_wheel_velocity_publischer = this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_wheel_back_velocity_controller/command",1);
    this->m_right_rear_wheel_velocity_publischer = this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_wheel_back_velocity_controller/command",1);
    this->m_left_front_wheel_velocity_publischer = this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_wheel_front_velocity_controller/command",1);
    this->m_right_front_wheel_velocity_publischer = this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_wheel_front_velocity_controller/command",1);

    this->m_left_steering_position_publischer = this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_steering_hinge_position_controller/command",1);
    this->m_right_steering_position_publischer = this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_steering_hinge_position_controller/command",1);
}

/**
 * @brief Construct a new Drive Param Converter::convert Drive Parameters Callback object
 * 
 * @param parameters 
 */
void DriveParamConverter::convertDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    std_msgs::Float64 throttle;
    throttle.data = parameters->velocity /0.1;

    //Throttle
    this->m_left_rear_wheel_velocity_publischer.publish(throttle);
    this->m_right_rear_wheel_velocity_publischer.publish(throttle);
    this->m_left_front_wheel_velocity_publischer.publish(throttle);
    this->m_right_front_wheel_velocity_publischer.publish(throttle);

    //Angles
    Angles angles = calculateSteeringAngles(parameters->angle);

    std_msgs::Float64 left_wheel;
    left_wheel.data = angles.angle1;

    std_msgs::Float64 right_wheel;
    right_wheel.data = angles.angle2;

    this->m_left_steering_position_publischer.publish(left_wheel);
    this->m_right_steering_position_publischer.publish(right_wheel);
}

/**
 * @brief 
 * 
 * @param angle 
 * @return Angles 
 */
DriveParamConverter::Angles DriveParamConverter::calculateSteeringAngles(const double& angle)
{
    Angles angles;
    double radius = tan(angle+M_PI/2)*m_length;
    angles.angle1=-atan(m_length/(radius+m_width/2));//left wheel
    angles.angle2=-atan(m_length/(radius-m_width/2));//right wheel
    return angles;
}
