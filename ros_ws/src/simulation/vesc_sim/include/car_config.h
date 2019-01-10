#pragma once

#include <cmath>

namespace car_config
{
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
    constexpr double WHEEL_PERIMETER{ WHEEL_DIAMETER * 3.14159265358979323846 }; // PI

    /**
     * @brief Minimal turning radius of car
     * @unit m
     */
    constexpr double CAR_MIN_RADIUS{ 0.605 };

    /**
     * @brief
     * @unit 1/min
     */
    constexpr double MAX_RPM_MECHANICAL{ 60000 };

    /**
     * @brief number of electrical motor poles
     * @unit none
     */
    constexpr double MOTOR_POLES{ 3 };

    /**
     * @brief
     * @unit 1/min
     */
    constexpr double MAX_RPM_ELECTRICAL{ MAX_RPM_MECHANICAL / MOTOR_POLES };

    /**
     * @brief conversion gain from electrical rounds per minute to meter per second
     * @unit m/s * minute
     */
    constexpr double ERPM_TO_SPEED{ WHEEL_PERIMETER * MOTOR_POLES / 60 }; // 60 seconds per minute

    /**
     * @brief conversion gain from meter per second to electrical rounds per minute
     * @unit s/ (m * minute)
     */
    constexpr double SPEED_TO_ERPM{ 1 / ERPM_TO_SPEED };

    /**
     * @brief
     * @unit
     */
    constexpr double SPEED_TO_ERPM_OFFSET{ 0 };

    /**
     * @brief conversion gain from mechanical rounds per minute to meter per second
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
    constexpr double STEERING_TO_SERVO_GAIN{ -3.14159265358979323846 / 3 };

    /**
     * @brief
     * @unit radian
     */
    constexpr double MAX_STEERING_ANGLE{ 3.14159265358979323846 / 6 }; // 30°

    /**
     * @brief
     * @unit radian
     */
    constexpr double MIN_STEERING_ANGLE{ -3.14159265358979323846 / 6 }; //-30°

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
     * @brief conversion gain from electrical rounds per minute to radian to seconds
     * @unit radian/s * minute
     */
    constexpr double ERPM_TO_RAD_PER_SEC{ MOTOR_POLES * 2 * 3.14159265358979323846 / 60 }; // 60 seconds per minute
};