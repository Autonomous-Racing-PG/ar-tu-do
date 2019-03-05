#pragma once

namespace car_config
{
    /**
     * @brief Since it's not possible to use <cmath> in a constexpression, it's defined here again.
     */
    constexpr double PI = 3.14159265358979323846;

    constexpr double DEG_TO_RAD = PI / 180;

    /**
     * @brief The distance between the front and rear axes
     */
    constexpr double WHEELBASE = 0.325;

    constexpr double WHEEL_DIAMETER = 0.098;

    constexpr double WHEEL_WIDTH = 0.042;

    constexpr double FRONT_WHEEL_DISTANCE = 0.23;

    constexpr double REAR_WHEEL_DISTANCE = 0.233;

    constexpr double WHEEL_PERIMETER = WHEEL_DIAMETER * PI;

    constexpr double TURNING_RADIUS = 0.605;

    /**
     * @brief Maximum revolutions per minute of the motor. Divide by TRANSMISSION to get the maximum wheel rpm
     * @unit 1/min
     */
    constexpr double MAX_RPM_MECHANICAL = 60000;

    /**
     * @brief Number of electrical motor poles
     */
    constexpr double MOTOR_POLES = 3;

    /**
     * @brief Maximum electrical revolutions per minute for use in the VESC
     * @unit 1/min
     */
    constexpr double MAX_RPM_ELECTRICAL = MAX_RPM_MECHANICAL / MOTOR_POLES;

    /**
     * @brief Conversion factor from electrical revolutions per minute to meters per second
     * @unit m/s * minute = m
     */
    constexpr double ERPM_TO_SPEED = WHEEL_PERIMETER * MOTOR_POLES / 60;

    /**
     * @brief Conversion factor from meters per second to electrical revolutions per minute
     * @unit s/ (m * minute) = 1/m
     */
    constexpr double SPEED_TO_ERPM = 1 / ERPM_TO_SPEED;

    /**
     * @brief Conversion factor from mechanical revolutions per minute to meters per second
     * @unit m/s * minute = m
     */
    constexpr double RPM_TO_SPEED = WHEEL_PERIMETER / 60;

    /**
     * @brief Position of the servo for normal wheel position
     */
    constexpr double STEERING_TO_SERVO_OFFSET = 0.5;

    /**
     * @brief Conversion factor from steering angle to servo input
     * @unit 1/radian
     */
    constexpr double STEERING_TO_SERVO_GAIN = -3 / PI;

    constexpr double MAX_STEERING_ANGLE = 30 * DEG_TO_RAD;

    constexpr double MIN_STEERING_ANGLE = -30 * DEG_TO_RAD;

    /**
     * @brief Gear transmission inside the differential of the car.
     * This is an estimate, the exact value is not known.
     */
    constexpr double TRANSMISSION = 20;

    constexpr double MAX_SERVO_POSITION = 1;

    /**
     * @brief Conversion factor from electrical revolutions per minute to radian to seconds
     * @unit radian/s * minute = radians
     */
    constexpr double ERPM_TO_RAD_PER_SEC = MOTOR_POLES * 2 * PI / 60;

    constexpr char COMMAND_POSITION[]{ "/commands/servo/position" };
    constexpr char COMMAND_THROTTLE[]{ "/commands/motor/speed" };
    constexpr char COMMAND_BRAKE[]{ "/commands/motor/brake" };

    namespace simulation
    {
        constexpr char WHEEL_LEFT_BACK_VELOCITY[]{ "/racer/left_wheel_back_velocity_controller/command" };
        constexpr char WHEEL_LEFT_FRONT_VELOCITY[]{ "/racer/left_wheel_front_velocity_controller/command" };
        constexpr char WHEEL_RIGHT_BACK_VELOCITY[]{ "/racer/right_wheel_back_velocity_controller/command" };
        constexpr char WHEEL_RIGHT_FRONT_VELOCITY[]{ "/racer/right_wheel_front_velocity_controller/command" };
        constexpr char LEFT_STEERING_POSITION[]{ "/racer/left_steering_hinge_position_controller/command" };
        constexpr char RIGHT_STEERING_POSITION[]{ "/racer/right_steering_hinge_position_controller/command" };
    };

    constexpr const char TOPIC_DRIVE_PARAM[]{ "/set/drive_param" };
    constexpr const char CMD_VEL[]{ "cmd_vel" };
};