#pragma once

namespace car_config
{
    /**
     * @brief It is not possible to use <cmath> PI definition in constexpr.
     * That is why it is defined here again.
     */
    constexpr double PI{ 3.14159265358979323846 };

    /**
     * @brief Distance between front and rear axis, wheelbase
     * @unit m
     */
    constexpr double WHEELBASE{ 0.325 };

    /**
     * @brief Distance between center of rear wheels suspension
     * @unit m
     */
    constexpr double REAR_WHEEL_DISTANCE{ 0.233 };

    /**
     * @brief Diameter of all wheels
     * @unit m
     */
    constexpr double WHEEL_DIAMETER{ 0.098 };

    /**
     * @brief Width of all wheels
     * @unit m
     */
    constexpr double WHEEL_WIDTH{ 0.042 };

    /**
     * @brief Distance between center of front wheels suspension
     * @unit m
     */
    constexpr double FRONT_WHEEL_DISTANCE{ 0.23 };

    /**
     * @brief Perimeter of all wheels
     * @unit m
     */
    constexpr double WHEEL_PERIMETER{ WHEEL_DIAMETER * PI };

    /**
     * @brief Minimal turning radius of car
     * @unit m
     */
    constexpr double TURNING_RADIUS{ 0.605 };

    /**
     * @brief revolutions per miunte of motor, divide through transmission to get wheel rpm
     * @unit 1/min
     */
    constexpr double MAX_RPM_MECHANICAL{ 60000 };

    /**
     * @brief number of electrical motor poles
     * @unit none
     */
    constexpr double MOTOR_POLES{ 3 };

    /**
     * @brief vesc calculates in electrical revolutions per minute
     * @unit 1/min
     */
    constexpr double MAX_RPM_ELECTRICAL{ MAX_RPM_MECHANICAL / MOTOR_POLES };

    /**
     * @brief conversion gain from electrical revolutions per minute to meter per second
     * @unit m/s * minute
     */
    constexpr double ERPM_TO_SPEED{ WHEEL_PERIMETER * MOTOR_POLES / 60 }; // 60 seconds per minute

    /**
     * @brief conversion gain from meter per second to electrical revolutions per minute
     * @unit s/ (m * minute)
     */
    constexpr double SPEED_TO_ERPM{ 1 / ERPM_TO_SPEED };

    /**
     * @brief conversion gain from mechanical revolutions per minute to meter per second
     * @unit m/s * minute
     */
    constexpr double RPM_TO_SPEED{ WHEEL_PERIMETER / 60 };

    /**
     * @brief offset of the servo for normal wheel position
     * @unit none
     */
    constexpr double STEERING_TO_SERVO_OFFSET{ 0.5 };

    /**
     * @brief conversion gain from steering angle to servo input
     * @unit 1/radian
     */
    constexpr double STEERING_TO_SERVO_GAIN{ -PI / 3 };

    /**
     * @brief
     * @unit radian
     */
    constexpr double MAX_STEERING_ANGLE{ PI / 6 }; // 30°

    /**
     * @brief
     * @unit radian
     */
    constexpr double MIN_STEERING_ANGLE{ -PI / 6 }; //-30°

    /**
     * @brief gear transmission inside the differential of the car
     * @unit none
     */
    constexpr double TRANSMISSION{ 20 }; // do not know exactly

    /**
     * @brief
     * @unit none
     */
    constexpr double MAX_SERVO_POSITION{ 1 };

    /**
     * @brief conversion gain from electrical revolutions per minute to radian to seconds
     * @unit radian/s * minute
     */
    constexpr double ERPM_TO_RAD_PER_SEC{ MOTOR_POLES * 2 * PI / 60 }; // 60 seconds per minute

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
};