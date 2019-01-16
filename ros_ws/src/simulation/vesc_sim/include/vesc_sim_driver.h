#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>

#include "vesc_sim.h"

/**
 * @brief Class to convert Drive Parameter Messages into single messages
 *
 * Class to convert Drive Parameter Messages (steering angle and velocity)
 * into single messages for each wheel velocity and for front wheel steering angles
 * based on Ackermann equations.
 */
class VESCSimulationDriver
{
    public:
    /**
     * @brief Constructor that creates subscribers and publishers
     *
     */
    VESCSimulationDriver();

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param motor_speed contains electrical RPM
     */
    void motorSpeedCallback(const std_msgs::Float64::ConstPtr& throttle_message);

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param servo_position contains angle with value 0 upto 1
     */
    void servoPositionCallback(const std_msgs::Float64::ConstPtr& servo_position);

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param motor_brake
     */
    void motorBrakeCallback(const std_msgs::Float64::ConstPtr& motor_brake);

    private:
    /**
     * @brief Return type to return two angles as return value
     *
     */
    struct AckermannSteeringAngles
    {
        double left_wheel_angle;
        double right_wheel_angle;
    };

    /**
     * @brief Calculates the angles of the front wheels based on the angle of the center of the front axis with
     * Ackermann equation/trigonometry
     *
     * @param angle Angle of the center of the front axis
     * @return Angles One Ackermann angle for each front wheel
     */
    AckermannSteeringAngles calculateSteeringAngles(const double& angle);

    /**
     * @brief ROS Handle
     *
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief ROS Subscriber for servo position messages
     * TOPIC: "/commands/servo/position"
     */
    ros::Subscriber m_servo_position_subscriber;

    /**
     * @brief ROS Subscriber for motor speed messages
     * TOPIC: "/commands/motor/speed"
     */
    ros::Subscriber m_motor_speed_subscriber;

    /**
     * @brief ROS Subscriber for brake messages
     * TOPIC: "/commands/motor/brake"
     */
    ros::Subscriber m_motor_brake_subscriber;

    /**
     * @brief ROS Publischer for the velocity of left rear wheel
     *
     */
    ros::Publisher m_left_rear_wheel_velocity_publisher;

    /**
     * @brief ROS Publischer for the velocity of right rear wheel
     *
     */
    ros::Publisher m_right_rear_wheel_velocity_publisher;
    /**
     * @brief ROS Publischer for the velocity of left front wheel
     *
     */
    ros::Publisher m_left_front_wheel_velocity_publisher;

    /**
     * @brief ROS Publischer for the velocity of right front wheel
     *
     */
    ros::Publisher m_right_front_wheel_velocity_publisher;

    /**
     * @brief ROS Publisher for the steering angle of left front wheel
     *
     */
    ros::Publisher m_left_steering_position_publisher;

    /**
     * @brief ROS Publisher for the steering angle of right front wheel
     *
     */
    ros::Publisher m_right_steering_position_publisher;

    VESCSimulator m_VESC_simulator;
};