#include "remote_keyboard.h"

RemoteKeyboard::RemoteKeyboard()
    : speed{ 0 }
    , angle{ 0 }
{
    out_drive_param =
        nh_.advertise< drive_msgs::drive_param >(TOPIC_DRIVE_PARAM, 1);
}

void RemoteKeyboard::keyLoop()
{
    std::cout << "listening to keyboard" << std::endl;
    std::cout << "=====================" << std::endl;

    while (ros::ok())
    {
        int c = getch();

        if (c == KEYCODE_W)
        {
            speed += 1;
        }

        if (c == KEYCODE_S)
        {
            speed -= 1;
        }

        if (c == KEYCODE_A)
        {
            angle += 1;
        }

        if (c == KEYCODE_D)
        {
            angle -= 1;
        }

        if (c == KEYCODE_SPACE)
        {
            std_msgs::Int64 msg;
            msg.data = (long)(ros::Time::now().toSec() * 1000);
            out_dms.publish(msg);
        }

        // after
        adjustDriveParam(speed, angle);
    }
}

int RemoteKeyboard::getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

void RemoteKeyboard::adjustDriveParam(double speed, double angle)
{
    drive_msgs::drive_param msg;
    msg.velocity = speed;
    msg.angle    = (angle + 1) / 2;
    out_drive_param.publish(msg);
}

void quit(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RemoteKeyboard");
    RemoteKeyboard remote_keyboard;

    std::cout << "listining to keyboard" << std::endl;
    std::cout << "=====================" << std::endl;

    signal(SIGINT, quit);
    remote_keyboard.keyLoop();
    return 0;
}
