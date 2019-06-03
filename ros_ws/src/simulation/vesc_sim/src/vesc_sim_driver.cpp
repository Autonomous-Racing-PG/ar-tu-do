#include "vesc_sim_driver.h"
#include "car_config.h"
#include <cmath>

VESCSimulationDriver::VESCSimulationDriver()
{
    this->m_servo_position_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>(COMMAND_POSITION, 1,
                                                         &VESCSimulationDriver::servoPositionCallback, this);

    this->m_motor_speed_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>(COMMAND_THROTTLE, 1, &VESCSimulationDriver::motorSpeedCallback,
                                                         this);

    this->m_motor_brake_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>(COMMAND_BRAKE, 1, &VESCSimulationDriver::motorBrakeCallback,
                                                         this);

    this->m_left_rear_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_LEFT_BACK_VELOCITY, 10);
    this->m_right_rear_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_RIGHT_BACK_VELOCITY, 10);
    this->m_left_front_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_LEFT_FRONT_VELOCITY, 10);
    this->m_right_front_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_RIGHT_FRONT_VELOCITY, 10);

    this->m_left_steering_position_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>(simulation::LEFT_STEERING_POSITION, 10);
    this->m_right_steering_position_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>(simulation::RIGHT_STEERING_POSITION, 10);

    m_VESC_simulator.start();
}

void VESCSimulationDriver::motorSpeedCallback(const std_msgs::Float64::ConstPtr& throttle_message)
{
    std_msgs::Float64 throttle;
    throttle.data = throttle_message->data * car_config::ERPM_TO_RAD_PER_SEC / car_config::TRANSMISSION;

    m_VESC_simulator.setSpeed(throttle_message->data);

    this->m_left_rear_wheel_velocity_publisher.publish(throttle);
    this->m_right_rear_wheel_velocity_publisher.publish(throttle);
    this->m_left_front_wheel_velocity_publisher.publish(throttle);
    this->m_right_front_wheel_velocity_publisher.publish(throttle);
}

void VESCSimulationDriver::motorBrakeCallback(const std_msgs::Float64::ConstPtr& motor_brake)
{
    if (motor_brake->data != 0)
    {
        std_msgs::Float64 throttle;
        throttle.data = 0;

        m_VESC_simulator.setSpeed(0);

        this->m_left_rear_wheel_velocity_publisher.publish(throttle);
        this->m_right_rear_wheel_velocity_publisher.publish(throttle);
        this->m_left_front_wheel_velocity_publisher.publish(throttle);
        this->m_right_front_wheel_velocity_publisher.publish(throttle);
    }
}

void VESCSimulationDriver::servoPositionCallback(const std_msgs::Float64::ConstPtr& servo_position)
{
    double angle = (servo_position->data - car_config::STEERING_TO_SERVO_OFFSET) / car_config::STEERING_TO_SERVO_GAIN;
    AckermannSteeringAngles angles = calculateSteeringAngles(angle);

    m_VESC_simulator.setServoAngle(servo_position->data);

    std_msgs::Float64 left_wheel;
    left_wheel.data = angles.left_wheel_angle;

    std_msgs::Float64 right_wheel;
    right_wheel.data = angles.right_wheel_angle;

    this->m_left_steering_position_publisher.publish(left_wheel);
    this->m_right_steering_position_publisher.publish(right_wheel);
}

VESCSimulationDriver::AckermannSteeringAngles VESCSimulationDriver::calculateSteeringAngles(const double& angle)
{
    AckermannSteeringAngles angles;
    double radius = std::tan(angle + M_PI / 2) * car_config::WHEELBASE;
    angles.left_wheel_angle = -std::atan(car_config::WHEELBASE / (radius + car_config::REAR_WHEEL_DISTANCE / 2));
    angles.right_wheel_angle = -std::atan(car_config::WHEELBASE / (radius - car_config::REAR_WHEEL_DISTANCE / 2));
    return angles;
}
