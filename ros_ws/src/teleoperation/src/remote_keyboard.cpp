#include "remote_keyboard.h"

RemoteKeyboard::RemoteKeyboard()
{
    this->drive_parameters_publisher =
        this->node_handle.advertise< drive_msgs::drive_param >(
            TOPIC_DRIVE_PARAMETERS, 1);
}

void RemoteKeyboard::keyboardLoop()
{
    std::cout << "Listening to keyboard..." << std::endl;
    std::cout << "========================" << std::endl;

    double velocity = 0;
    double angle    = 0;
    while (ros::ok())
    {
        int key = this->getKeyboardCharacter();

        if (key == KEYCODE_W)
        {
            velocity += 1;
        }

        if (key == KEYCODE_S)
        {
            velocity -= 1;
        }

        if (key == KEYCODE_A)
        {
            angle += 1;
        }

        if (key == KEYCODE_D)
        {
            angle -= 1;
        }

        this->publishDriveParameters(velocity, angle);
    }
}

int RemoteKeyboard::getKeyboardCharacter()
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

void RemoteKeyboard::publishDriveParameters(double velocity, double angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle    = (angle + 1) / 2;
    this->drive_parameters_publisher.publish(drive_parameters);
}

void quitSignalHandler(int signal)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remote_keyboard_controller");
    RemoteKeyboard remote_keyboard;

    signal(SIGINT, quitSignalHandler);
    remote_keyboard.keyboardLoop();
    return 0;
}
