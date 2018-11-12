# include <ros/ros.h>

# include <std_msgs/Float64.h>
# include <termios.h>
# include <signal.h>


# define TOPIC_SPEED "/set/speed"
# define TOPIC_ANGLE "/set/position"

# define KEYCODE_W 119
# define KEYCODE_A 97
# define KEYCODE_S 115
# define KEYCODE_D 100
# define KEYCODE_SPACE 32

class RemoteKeyboard


{
    public:
    RemoteKeyboard()
    void keyLoop()

    private:
    int getch()

    void adjustSpeed(double speed)
    void adjustAngle(double angle)

    ros:: NodeHandle nh_

    std:: string input

    ros:: Publisher  out_speed
    ros:: Publisher  out_angle
}

RemoteKeyboard: : RemoteKeyboard()
{
    out_speed = nh_.advertise < std_msgs:: Float64 > (TOPIC_SPEED, 1)
    out_angle = nh_.advertise < std_msgs:: Float64 > (TOPIC_ANGLE, 1)
}

void RemoteKeyboard: : keyLoop() {
    std:: cout << "listening to keyboard" << std: : endl
    std:: cout << "=====================" << std: : endl

    double speed = 0
    double angle = 0

    while (ros: : ok())
    {
        int c = getch()
        switch(c) {
            case KEYCODE_W:
            speed += 100
            break
            case KEYCODE_S:
            speed -= 100
            break
            case KEYCODE_A:
            angle -= 0.1
            break
            case KEYCODE_D:
            angle += 0.1
            break
            case KEYCODE_SPACE:
            speed = 0
            break
            default:
            break
        }
        adjustSpeed(speed)
        adjustAngle(angle)
    }
}


int RemoteKeyboard: : getch()
{
    static struct termios oldt, newt
    tcgetattr(STDIN_FILENO, & oldt)
    // save old settings
    newt = oldt
    newt.c_lflag &= ~(ICANON)
    // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, & newt)
    // apply new settings

    int c = getchar()
    // read character(non - blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, & oldt)
    // restore old settings
    return c
}

void RemoteKeyboard: : adjustSpeed(double speed) {
    std_msgs:: Float64 msg
    msg.data = speed
    out_speed.publish(msg)
}

void RemoteKeyboard: : adjustAngle(double angle) {
    std_msgs:: Float64 msg
    msg.data = (angle + 1) / 2
    out_angle.publish(msg)
}

void quit(int sig)
{
    ros:: shutdown()
    exit(0)
}

int main(int argc, char ** argv)
{
    ros:: init(argc, argv, "RemoteKeyboard")
    RemoteKeyboard remote_keyboard

    std:: cout << "listining to keyboard" << std: : endl
    std:: cout << "=====================" << std: : endl

    signal(SIGINT, quit)
    remote_keyboard.keyLoop()
    return 0
}
