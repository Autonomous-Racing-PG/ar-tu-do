#pragma once

#include <cmath>

namespace car_config
{
    /**
     * @brief Distance between front and rear axis, wheelbase
     * @unit m
     */
    constexpr double WHEELBASE{0.325};

    /**
     * @brief Distance between center of rear wheels suspension
     * @unit m
     */
    constexpr double REAR_WHEEL_DISTANCE{ 0.233 };

    /**
     * @brief Diameter of all wheels
     * @unit m
     */
    constexpr double WHEEL_DIAMETER{0.098};

    /**
     * @brief Width of all wheels
     * @unit m
     */
    constexpr double WHEEL_WIDTH{0.042};

    /**
     * @brief Distance between center of front wheels suspension
     * @unit m
     */
    constexpr double FRONT_WHEEL_DISTANCE{0.23};

    /**
     * @brief Perimeter of all wheels
     * @unit m
     */
    constexpr double WHEEL_PERIMETER{WHEEL_DIAMETER*3.14159265358979323846}; //PI

    /**
     * @brief Minimal turning radius of car
     * @unit m
     */
    constexpr double CAR_MIN_RADIUS{0.605};

    /**
     * @brief 
     * @unit 1/min
     */
    constexpr double MAX_RPM_MECHANICAL{60000};

    /**
     * @brief 
     * @unit none
     */
    constexpr double MOTOR_POLES{3};
    
    /**
     * @brief 
     * @unit 1/min
     */
    constexpr double MAX_RPM_ELECTRICAL{MAX_RPM_MECHANICAL/MOTOR_POLES};

    constexpr double ERPM_TO_SPEED{WHEEL_PERIMETER*MOTOR_POLES/60};

    constexpr double SPEED_TO_ERPM{1/ERPM_TO_SPEED};

    constexpr double SPEED_TO_ERPM_OFFSET{0};

    constexpr double RPM_TO_SPEED{WHEEL_PERIMETER/60};

    constexpr double STEERING_TO_SERVO_OFFSET{0.5};

    constexpr double STEERING_TO_SERVO_GAIN{-3.14159265358979323846/3};

    constexpr double MAX_STEERING_ANGLE{3.14159265358979323846/6};//30°

    constexpr double MIN_STEERING_ANGLE{-3.14159265358979323846/6};//-30°
};