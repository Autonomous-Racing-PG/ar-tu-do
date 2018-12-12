#include "keyboard_controller.h"

KeyboardController::KeyboardController()
{
    this->drive_parameters_publisher = this->node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

void KeyboardController::keyboardLoop()
{
    std::cout << "Listening to keyboard..." << std::endl;
    std::cout << "========================" << std::endl;

    double velocity = 0;
    double angle = 0;
    while (ros::ok())
    {
        auto key = static_cast<Keycode>(this->getKeyboardCharacter());

        switch (key)
        {
            case Keycode::W:
                velocity += 1;
                break;
            case Keycode::S:
                velocity -= 1;
                break;
            case Keycode::A:
                angle += 1;
                break;
            case Keycode::D:
                angle -= 1;
                break;
        }

        this->publishDriveParameters(velocity, angle);
    }
}

int KeyboardController::getKeyboardCharacter()
{
    static struct termios old_terminal, new_terminal;
    // back up current terminal settings
    tcgetattr(STDIN_FILENO, &old_terminal);
    new_terminal = old_terminal;
    // disable buffering
    new_terminal.c_lflag &= ~ICANON;
    // apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);

    // read character (non-blocking)
    int character = getchar();

    // restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);

    return character;
}

void KeyboardController::publishDriveParameters(double velocity, double angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = (angle + 1) / 2;
    this->drive_parameters_publisher.publish(drive_parameters);
}

void quitSignalHandler(int signal)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_controller");
    KeyboardController keyboard_controller;

    signal(SIGINT, quitSignalHandler);
    keyboard_controller.keyboardLoop();
    return EXIT_SUCCESS;
}
