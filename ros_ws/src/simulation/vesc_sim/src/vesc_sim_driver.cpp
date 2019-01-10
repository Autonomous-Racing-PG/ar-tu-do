#include "vesc_sim_driver.h"
#include "car_config.h"

VESCSimDriver::VESCSimDriver()
{
    this->m_servo_position_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>("/commands/servo/position", 1,
                                                         &VESCSimDriver::servoPositionCallback, this);

    this->m_motor_speed_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>("/commands/motor/speed", 1, &VESCSimDriver::motorSpeedCallback,
                                                         this);

    this->m_motor_break_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>("/commands/motor/break", 1, &VESCSimDriver::motorBreakCallback,
                                                         this);

    this->m_left_rear_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_wheel_back_velocity_controller/command", 10);
    this->m_right_rear_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_wheel_back_velocity_controller/command", 10);
    this->m_left_front_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_wheel_front_velocity_controller/command", 10);
    this->m_right_front_wheel_velocity_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_wheel_front_velocity_controller/command", 10);

    this->m_left_steering_position_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/left_steering_hinge_position_controller/command", 10);
    this->m_right_steering_position_publisher =
        this->m_node_handle.advertise<std_msgs::Float64>("/racer/right_steering_hinge_position_controller/command", 10);

    m_simulator.start();
}

void VESCSimDriver::motorSpeedCallback(const std_msgs::Float64::ConstPtr& motor_speed)
{
    std_msgs::Float64 throttle;
    throttle.data = (motor_speed->data - car_config::SPEED_TO_ERPM_OFFSET) * car_config::ERPM_TO_RAD_PER_SEC /
        car_config::TRANSMISSION;

    m_simulator.setSpeed(motor_speed->data);

    this->m_left_rear_wheel_velocity_publisher.publish(throttle);
    this->m_right_rear_wheel_velocity_publisher.publish(throttle);
    this->m_left_front_wheel_velocity_publisher.publish(throttle);
    this->m_right_front_wheel_velocity_publisher.publish(throttle);
}

void VESCSimDriver::motorBreakCallback(const std_msgs::Float64::ConstPtr& motor_break)
{
    if (motor_break->data != 0)
    {
        std_msgs::Float64 throttle;
        throttle.data = 0;

        m_simulator.setSpeed(0);

        this->m_left_rear_wheel_velocity_publisher.publish(throttle);
        this->m_right_rear_wheel_velocity_publisher.publish(throttle);
        this->m_left_front_wheel_velocity_publisher.publish(throttle);
        this->m_right_front_wheel_velocity_publisher.publish(throttle);
    }
}

void VESCSimDriver::servoPositionCallback(const std_msgs::Float64::ConstPtr& servo_position)
{
    double angle = (servo_position->data - car_config::STEERING_TO_SERVO_OFFSET) / car_config::STEERING_TO_SERVO_GAIN;
    AckermannSteeringAngles angles = calculateSteeringAngles(angle);

    m_simulator.setServo(servo_position->data);

    std_msgs::Float64 left_wheel;
    left_wheel.data = angles.left_wheel_angle;

    std_msgs::Float64 right_wheel;
    right_wheel.data = angles.right_wheel_angle;

    this->m_left_steering_position_publisher.publish(left_wheel);
    this->m_right_steering_position_publisher.publish(right_wheel);
}

VESCSimDriver::AckermannSteeringAngles VESCSimDriver::calculateSteeringAngles(const double& angle)
{
    AckermannSteeringAngles angles;
    double radius = tan(angle + M_PI / 2) * car_config::WHEELBASE;
    angles.left_wheel_angle =
        -atan(car_config::WHEELBASE / (radius + car_config::REAR_WHEEL_DISTANCE / 2)); // left wheel
    angles.right_wheel_angle =
        -atan(car_config::WHEELBASE / (radius - car_config::REAR_WHEEL_DISTANCE / 2)); // right wheel
    return angles;
}
