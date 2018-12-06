#include "remote_keyboard.h"

RemoteKeyboard::RemoteKeyboard()
{
    this->driveParametersPublisher =
        this->nodeHandle.advertise< drive_msgs::drive_param >(TOPIC_DRIVE_PARAMETERS, 1);
}

void RemoteKeyboard::keyboardLoop()
{
    std::cout << "Listening to keyboard..." << std::endl;
    std::cout << "========================" << std::endl;

    double velocity = 0;
    double angle = 0;
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

        // TODO Implement Dead Man's Switch
        /*if (key == KEYCODE_SPACE)
        {
            std_msgs::Int64 deadMansSwitchMessage;
            deadMansSwitchMessage.data = (long)(ros::Time::now().toSec() * 1000);
            this->deadMansSwitchPublisher.publish(deadMansSwitchMessage);
        } */

        publishDriveParameters(velocity, angle);
    }
}

int RemoteKeyboard::getKeyboardCharacter()
{
    static struct termios oldTerminal, newTerminal;
    // back up current terminal settings
    tcgetattr(STDIN_FILENO, &oldTerminal);
    newTerminal = oldTerminal;
    // disable buffering
    newTerminal.c_lflag &= ~ICANON;
    // apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newTerminal); 

    // read character (non-blocking)
    int character = getchar();

    // restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldTerminal);

    return character;
}

void RemoteKeyboard::publishDriveParameters(double velocity, double angle)
{
    drive_msgs::drive_param driveParameters;
    driveParameters.velocity = velocity;
    driveParameters.angle = (angle + 1) / 2;
    this->driveParametersPublisher.publish(driveParameters);
}

void quitSignalHandler(int signal)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remote_keyboard_controller");
    RemoteKeyboard remoteKeyboard;

    signal(SIGINT, quitSignalHandler);
    remoteKeyboard.keyboardLoop();
    return 0;
}
