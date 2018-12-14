#include "keyboard_controller.h"

double clamp(double value, double lower, double upper)
{
    return min(upper, max(value, lower));
}

double map(double value, double in_lower, double in_upper, double out_lower, double out_upper)
{
    return out_lower + (out_upper - out_lower) * (value - in_lower) / (in_upper - in_lower);
}

KeyboardController::KeyboardController()
{
    this->m_drive_parameters_publisher =
        this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
    this->createWindow();
}

KeyboardController::~KeyboardController()
{
    SDL_DestroyWindow(this->m_window);
    SDL_Quit();
}

void KeyboardController::createWindow()
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        throw std::runtime_error("Could not initialize SDL");
    }
    this->m_window = SDL_CreateWindow("Keyboard teleoperation - Use WASD keys", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 500, 150, SDL_WINDOW_RESIZABLE);
}

void KeyboardController::pollKeyboardEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYUP || event.type == SDL_KEYDOWN)
        {
            for (int i = 0; i < KEY_COUNT; i++)
            {
                if ((uint16_t)event.key.keysym.sym == (int)KEY_CODES[i])
                {
                    this->m_key_pressed_state[i] = event.type == SDL_KEYDOWN;
                }
            }
        }
        else if (event.type == SDL_QUIT)
        {
            ros::shutdown();
            exit(EXIT_SUCCESS);
        }
    }
}

void KeyboardController::keyboardLoop()
{
    ros::Rate timer(PARAMETER_UPDATE_FREQUENCY);
    double delta_time = timer.expectedCycleTime().toSec();

    while (ros::ok())
    {
        this->pollKeyboardEvents();
        this->updateDriveParameters(delta_time);
        this->publishDriveParameters();

        ros::spinOnce();
        timer.sleep();
    }
}

void KeyboardController::updateDriveParameters(double delta_time)
{
    double steer = this->m_key_pressed_state[(int)KeyIndex::STEER_LEFT]
        ? +1
        : (this->m_key_pressed_state[(int)KeyIndex::STEER_RIGHT] ? -1 : 0);
    double throttle = this->m_key_pressed_state[(int)KeyIndex::ACCELERATE]
        ? +1
        : (this->m_key_pressed_state[(int)KeyIndex::DECELERATE] ? -1 : 0);

    double steer_limit = map(fabs(this->m_velocity), 0, MAX_VELOCITY, 1, FAST_STEER_LIMIT);
    double angle_update = steer * delta_time * STEERING_SPEED;
    this->m_angle = clamp(this->m_angle + angle_update, -MAX_STEERING * steer_limit, +MAX_STEERING * steer_limit);
    double velocity_update = throttle * delta_time * (this->m_velocity * throttle > 0 ? ACCELERATION : BRAKING);
    this->m_velocity = clamp(this->m_velocity + velocity_update, -MAX_VELOCITY, +MAX_VELOCITY);

    if (steer == 0 && this->m_angle != 0)
    {
        double sign = this->m_angle > 0 ? 1 : -1;
        this->m_angle -= STEERING_GRAVITY * delta_time * sign;
        if (fabs(this->m_angle) < STEERING_GRAVITY * delta_time)
        {
            this->m_angle = 0;
        }
    }

    if (throttle == 0 && this->m_velocity != 0)
    {
        double sign = this->m_velocity > 0 ? 1 : -1;
        this->m_velocity -= THROTTLE_GRAVITY * delta_time * sign;
        if (fabs(this->m_velocity) < THROTTLE_GRAVITY * delta_time)
        {
            this->m_velocity = 0;
        }
    }
}

void KeyboardController::publishDriveParameters()
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = this->m_velocity;
    drive_parameters.angle = this->m_angle;
    this->m_drive_parameters_publisher.publish(drive_parameters);
}

void quitSignalHandler(int)
{
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_controller");
    KeyboardController keyboard_controller;

    signal(SIGINT, quitSignalHandler);
    keyboard_controller.keyboardLoop();
    return EXIT_SUCCESS;
}
