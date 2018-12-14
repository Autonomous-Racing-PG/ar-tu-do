#pragma once

// Ignore some warnings from external headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

#include <SDL2/SDL.h>
#include <algorithm>
#include <array>
#include <drive_msgs/drive_param.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdexcept>

// Re-Enable the disabled warnings
#pragma GCC diagnostic pop


constexpr const char* TOPIC_DRIVE_PARAMETERS = "/set/drive_param";

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
    STEER_RIGHT = 3
};

constexpr int KEY_COUNT = 4;

constexpr std::array<Keycode, KEY_COUNT> KEY_CODES = { Keycode::W, Keycode::A, Keycode::S, Keycode::D };

constexpr double PARAMETER_UPDATE_FREQUENCY = 90;

// How fast the steering value changes, in units per second
constexpr double STEERING_SPEED = 2.5;
// How fast the velocity changes, in units per second
constexpr double ACCELERATION = 3;
// How fast the velocity changes when decelerating, in units per second
constexpr double BRAKING = 8;

// The steering angle is clamped so that its absolute value is not greater than this
constexpr double MAX_STEERING = 0.7;
// MAX_STEERING is multiplied by this when travelling at MAX_VELOCITY, by 1.0 when resting and by an interpolated value
// otherwise
constexpr double FAST_STEER_LIMIT = 0.6;
// The velocity is clamped so that its absolute value is not greater than this
constexpr double MAX_VELOCITY = 8;

// When no steering key is pressed, the steering value will change towards 0 at this rate, in units per second
constexpr double STEERING_GRAVITY = 2;
// When no throttle key is pressed, the velocity will change towards 0 at this rate, in units per second
constexpr double THROTTLE_GRAVITY = 3;

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

    SDL_Window* m_window;

    std::array<bool, KEY_COUNT> m_key_pressed_state = { { false, false, false, false } };

    double m_velocity = 0;
    double m_angle = 0;

    ros::Timer m_timer;

    void pollWindowEvents();

    void updateDriveParameters(double delta_time);

    void publishDriveParameters();

    void createWindow();
    void timerCallback(const ros::TimerEvent& event);
};
