#pragma once

#include <SDL2/SDL.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <drive_msgs/drive_param.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <stdexcept>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "input/drive_param/keyboard";
constexpr const char* TOPIC_DEAD_MANS_SWITCH = "/input/dms_heartbeat";
constexpr const char* TOPIC_UNLOCK_MOTOR = "/commands/unlock_motor";

enum class Keycode : int
{
    W = 119,
    A = 97,
    S = 115,
    D = 100,
    SPACE = 32
};

enum class KeyIndex : int
{
    ACCELERATE = 0,
    DECELERATE = 2,
    STEER_LEFT = 1,
    STEER_RIGHT = 3,
    DEAD_MANS_SWITCH = 4
};

constexpr int KEY_COUNT = 5;

constexpr std::array<Keycode, KEY_COUNT> KEY_CODES = { Keycode::W, Keycode::A, Keycode::S, Keycode::D, Keycode::SPACE };

constexpr double PARAMETER_UPDATE_FREQUENCY = 90;

// How fast the steering value changes, in units per second
constexpr double STEERING_SPEED = 6;
// How fast the velocity changes, in units per second
constexpr double ACCELERATION = 0.4;
// How fast the velocity changes when decelerating, in units per second
constexpr double BRAKING = 2;

// MAX_STEERING is multiplied by this when travelling at maximum velocity, by 1.0 when resting and by an interpolated
// value otherwise
constexpr double FAST_STEER_LIMIT = 0.6;

// When no steering key is pressed, the steering value will change towards 0 at this rate, in units per second
constexpr double STEERING_GRAVITY = 2;
// When no throttle key is pressed, the velocity will change towards 0 at this rate, in units per second
constexpr double THROTTLE_GRAVITY = 3;

constexpr double MAX_THROTTLE = 0.25;

class KeyboardController
{
    public:
    KeyboardController();
    KeyboardController(KeyboardController&&) = default;
    KeyboardController(const KeyboardController&) = default;
    ~KeyboardController();

    private:
    ros::NodeHandle m_node_handle;

    ros::Publisher m_drive_parameters_publisher;
    ros::Publisher m_dead_mans_switch_publisher;

    ros::Subscriber m_unlock_motor_subscriber;

    SDL_Window* m_window;

    std::array<bool, KEY_COUNT> m_key_pressed_state = { { false, false, false, false } };

    double m_velocity = 0;
    double m_angle = 0;

    ros::Timer m_timer;

    bool m_car_unlocked = false;

    void pollWindowEvents();

    void updateDriveParameters(double delta_time);

    void publishDriveParameters();

    void updateDeadMansSwitch();

    void createWindow();
    void timerCallback(const ros::TimerEvent& event);
    void unlockMotorCallback(const std_msgs::Bool::ConstPtr& unlock_motor_message);
    void updateWindowColor();
};
