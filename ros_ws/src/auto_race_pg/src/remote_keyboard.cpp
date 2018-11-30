#include "remote_keyboard.h"

RemoteKeyboard::RemoteKeyboard()
    : speed{ 0 }
    , angle{ 0 }
{
    out_speed = nh_.advertise< std_msgs::Float64 >(TOPIC_SPEED, 1);
    out_angle = nh_.advertise< std_msgs::Float64 >(TOPIC_ANGLE, 1);
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
        adjustSpeed(speed);
        adjustAngle(angle);
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

void RemoteKeyboard::adjustSpeed(double speed)
{
    std_msgs::Float64 msg;
    msg.data = speed;
    out_speed.publish(msg);
}

void RemoteKeyboard::adjustAngle(double angle)
{
    std_msgs::Float64 msg;
    msg.data = (angle + 1) / 2;
    out_angle.publish(msg);
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
