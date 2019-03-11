#include "keyboard_controller.h"
#include <std_msgs/Int64.h>
using std::abs;

double clamp(double value, double lower, double upper)
{
    return std::min(upper, std::max(value, lower));
}

double map(double value, double in_lower, double in_upper, double out_lower, double out_upper)
{
    return out_lower + (out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower);
}

/**
 * Class constructor that sets up a publisher for the drive parameters topic, creates a window and starts a timer for
 * the main loop
 * */
KeyboardController::KeyboardController()
{
    ROS_ASSERT_MSG(KEY_CODES.size() == KEY_COUNT, "KEY_CODES needs to have KEY_COUNT many elements.");
    ROS_ASSERT_MSG(this->m_key_pressed_state.size() == KEY_COUNT,
                   "m_key_pressed_state needs to have KEY_COUNT many elements.");

    this->m_drive_parameters_publisher =
        this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
    this->m_dead_mans_switch_publisher = this->m_node_handle.advertise<std_msgs::Int64>(TOPIC_DEAD_MANS_SWITCH, 1);
    this->m_unlock_motor_subscriber =
        this->m_node_handle.subscribe<std_msgs::Bool>(TOPIC_UNLOCK_MOTOR, 1, &KeyboardController::unlockMotorCallback,
                                                      this);

    this->createWindow();

    auto tick_duration = ros::Duration(1.0 / PARAMETER_UPDATE_FREQUENCY);
    this->m_timer = this->m_node_handle.createTimer(tick_duration, &KeyboardController::timerCallback, this);
}

KeyboardController::~KeyboardController()
{
    SDL_DestroyWindow(this->m_window);
    SDL_Quit();
}

void KeyboardController::createWindow()
{
    std::string icon_filename = ros::package::getPath("teleoperation") + std::string("/wasd.bmp");

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        throw std::runtime_error("Could not initialize SDL: " + std::string(SDL_GetError()));
    }
    this->m_window = SDL_CreateWindow("Keyboard teleoperation - Use WASD keys", SDL_WINDOWPOS_UNDEFINED,
                                      SDL_WINDOWPOS_UNDEFINED, 450, 100, SDL_WINDOW_RESIZABLE);

    SDL_Surface* icon = SDL_LoadBMP(icon_filename.c_str());
    if (icon != NULL)
    {
        SDL_SetWindowIcon(this->m_window, icon);
        SDL_FreeSurface(icon);
    }
}

void KeyboardController::pollWindowEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYUP || event.type == SDL_KEYDOWN)
        {
            for (size_t i = 0; i < KEY_COUNT; i++)
            {
                if ((int)event.key.keysym.sym == (int)KEY_CODES[i])
                {
                    this->m_key_pressed_state[i] = event.type == SDL_KEYDOWN;
                }
            }
        }
        else if (event.type == SDL_QUIT)
        {
            ros::shutdown();
        }
        else if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_EXPOSED)
        {
            this->updateWindowColor();
        }
    }
}

void KeyboardController::updateWindowColor()
{
    SDL_Surface* surface = SDL_GetWindowSurface(this->m_window);
    if (this->m_car_unlocked)
    {
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 110, 199, 46));
    }
    else
    {
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0, 0, 0));
    }
    SDL_UpdateWindowSurface(this->m_window);
}

/**
 * This method is called at each tick of the timer. It updates the keyboard state and the drive parameters and publishes
 * them to the ROS topic.
 * */
void KeyboardController::timerCallback(const ros::TimerEvent& event)
{
    double delta_time = (event.current_real - event.last_real).toSec();

    this->pollWindowEvents();
    this->updateDriveParameters(delta_time);
    this->publishDriveParameters();
    this->updateDeadMansSwitch();
}

/**
 *  Checks if the Dead Man's Switch key is pressed and publish the Dead Man's Switch message
 */
void KeyboardController::updateDeadMansSwitch()
{
    if (this->m_key_pressed_state[(size_t)KeyIndex::DEAD_MANS_SWITCH])
    {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
        auto time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

        std_msgs::Int64 dead_mans_switch_message;
        dead_mans_switch_message.data = time_since_epoch.count();

        this->m_dead_mans_switch_publisher.publish(dead_mans_switch_message);
    }
}

/**
 *  This updates the drive parameters based on which keys are currently pressed and how much time passed since the last
 * update.
 *  If no keys are pressed, the steering angle and target velocity will move back to their default values over time.
 */
void KeyboardController::updateDriveParameters(double delta_time)
{
// Disable warnings about equality comparisons for floats.
// Equality comparisons are ok here because the variables are assigned the exact values that we compare them against.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    double steer = this->m_key_pressed_state[(size_t)KeyIndex::STEER_LEFT]
        ? -1
        : (this->m_key_pressed_state[(size_t)KeyIndex::STEER_RIGHT] ? +1 : 0);
    double throttle = this->m_key_pressed_state[(size_t)KeyIndex::ACCELERATE]
        ? +1
        : (this->m_key_pressed_state[(size_t)KeyIndex::DECELERATE] ? -1 : 0);

    double steer_limit = map(abs(this->m_velocity), 0, MAX_THROTTLE, 1, FAST_STEER_LIMIT);
    double angle_update = steer * delta_time * STEERING_SPEED;
    this->m_angle = clamp(this->m_angle + angle_update, -steer_limit, +steer_limit);
    double velocity_update = throttle * delta_time * (this->m_velocity * throttle > 0 ? ACCELERATION : BRAKING);
    this->m_velocity = clamp(this->m_velocity + velocity_update, -MAX_THROTTLE, +MAX_THROTTLE);

    if (steer == 0 && this->m_angle != 0)
    {
        double sign = copysign(1.0, this->m_angle);
        this->m_angle -= STEERING_GRAVITY * delta_time * sign;
        if (abs(this->m_angle) < STEERING_GRAVITY * delta_time)
        {
            this->m_angle = 0;
        }
    }

    if (throttle == 0 && this->m_velocity != 0)
    {
        double sign = copysign(1.0, this->m_velocity);
        this->m_velocity -= THROTTLE_GRAVITY * delta_time * sign;
        if (abs(this->m_velocity) < THROTTLE_GRAVITY * delta_time)
        {
            this->m_velocity = 0;
        }
    }
#pragma GCC diagnostic pop
}

void KeyboardController::publishDriveParameters()
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = this->m_velocity;
    drive_parameters.angle = this->m_angle;
    this->m_drive_parameters_publisher.publish(drive_parameters);
}

void KeyboardController::unlockMotorCallback(const std_msgs::Bool::ConstPtr& unlock_motor_message)
{
    if (this->m_car_unlocked != unlock_motor_message->data)
    {
        this->m_car_unlocked = unlock_motor_message->data;
        this->updateWindowColor();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_controller");
    KeyboardController keyboard_controller;
    ros::spin();
    return EXIT_SUCCESS;
}
