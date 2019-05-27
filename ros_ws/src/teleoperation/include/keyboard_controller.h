#pragma once

#include <SDL2/SDL.h>
#include <algorithm>
#include <array>
#include <drive_msgs/drive_param.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <stdexcept>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "input/drive_param/keyboard";
constexpr const char* TOPIC_HEARTBEAT_MANUAL = "/input/heartbeat_manual";
constexpr const char* TOPIC_HEARTBEAT_AUTONOMOUS = "/input/heartbeat_autonomous";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";

enum class Keycode : int
{
    W = 119,
    A = 97,
    S = 115,
    D = 100,
    SPACE = 32,
    B = 98
};

enum class KeyIndex : int
{
    ACCELERATE = 0,
    DECELERATE = 2,
    STEER_LEFT = 1,
    STEER_RIGHT = 3,
    ENABLE_MANUAL = 4,
    ENABLE_AUTONOMOUS = 5
};

constexpr int KEY_COUNT = 6;

constexpr std::array<Keycode, KEY_COUNT> KEY_CODES = { Keycode::W, Keycode::A,     Keycode::S,
                                                       Keycode::D, Keycode::SPACE, Keycode::B };

enum class DriveMode : int
{
    LOCKED = 0,
    MANUAL = 1,
    AUTONOMOUS = 2
};

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

constexpr double MAX_THROTTLE = 0.35;

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
    ros::Publisher m_enable_manual_publisher;
    ros::Publisher m_enable_autonomous_publisher;

    ros::Subscriber m_drive_mode_subscriber;

    SDL_Window* m_window;

    SDL_Surface* m_image_locked;
    SDL_Surface* m_image_manual;
    SDL_Surface* m_image_autonomous;

    std::array<bool, KEY_COUNT> m_key_pressed_state = { { false, false, false, false } };

    double m_velocity = 0;
    double m_angle = 0;

    ros::Timer m_timer;

    DriveMode m_drive_mode = DriveMode::LOCKED;

    void pollWindowEvents();

    void updateDriveParameters(double delta_time);

    void publishDriveParameters();

    void updateDeadMansSwitch();

    void createWindow();
    void timerCallback(const ros::TimerEvent& event);
    void driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message);
    void updateWindow();
    void loadImages();
};
