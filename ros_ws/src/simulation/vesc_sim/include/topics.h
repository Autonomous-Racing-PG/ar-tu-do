#pragma once

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";

namespace simulation
{
    constexpr char WHEEL_LEFT_BACK_VELOCITY[]{ "/racer/left_wheel_back_velocity_controller/command" };
    constexpr char WHEEL_LEFT_FRONT_VELOCITY[]{ "/racer/left_wheel_front_velocity_controller/command" };
    constexpr char WHEEL_RIGHT_BACK_VELOCITY[]{ "/racer/right_wheel_back_velocity_controller/command" };
    constexpr char WHEEL_RIGHT_FRONT_VELOCITY[]{ "/racer/right_wheel_front_velocity_controller/command" };
    constexpr char LEFT_STEERING_POSITION[]{ "/racer/left_steering_hinge_position_controller/command" };
    constexpr char RIGHT_STEERING_POSITION[]{ "/racer/right_steering_hinge_position_controller/command" };
};