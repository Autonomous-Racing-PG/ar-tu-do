#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <std_msgs/Float64.h>
#include <termios.h>

#define MAX_SPEED 5000
#define MAX_ANGLE 0.8

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/position"

#define KEYCODE_W 119
#define KEYCODE_A 97
#define KEYCODE_S 115
#define KEYCODE_D 100
#define KEYCODE_SPACE 32

class RemoteControl
{
    public:
    RemoteControl();
    void keyLoop();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    int getch();

    void adjustSpeed(double speed);
    void adjustAngle(double angle);

    ros::NodeHandle nh_;

    std::string input;

    double speed;
    double angle;

    int    linear_, angular_;
    double l_scale_, a_scale_;

    ros::Publisher out_speed;
    ros::Publisher out_angle;

    ros::Subscriber in_joy;
};

RemoteControl::RemoteControl()
    : linear_(1)
    , angular_(0)
    , speed(0)
    , angle(0)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    out_speed = nh_.advertise< std_msgs::Float64 >(TOPIC_SPEED, 1);
    out_angle = nh_.advertise< std_msgs::Float64 >(TOPIC_ANGLE, 1);

    in_joy =
        nh_.subscribe< sensor_msgs::Joy >("joy", 10,
                                          &RemoteControl::joyCallback, this);
}

void RemoteControl::keyLoop()
{
    std::cout << "listening to keyboard" << std::endl;
    std::cout << "=====================" << std::endl;
    while (ros::ok())
    {
        int c = getch();
        switch (c)
        {
            case KEYCODE_W:
                speed += 100;
                break;
            case KEYCODE_S:
                speed -= 100;
                break;
            case KEYCODE_A:
                angle -= 0.1;
                break;
            case KEYCODE_D:
                angle += 0.1;
                break;
            case KEYCODE_SPACE:
                speed = 0;
                break;
            default:
                break;
        }
        adjustSpeed(speed);
        adjustAngle(angle);
    }
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double speed = joy->axes[ linear_ ];
    double angle = joy->axes[ angular_ ];
    adjustSpeed(speed);
    adjustAngle(angle);
}

int RemoteControl::getch()
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

void RemoteControl::adjustSpeed(double speed)
{
    std_msgs::Float64 msg;
    msg.data = speed * MAX_SPEED;
    out_speed.publish(msg);
}

void RemoteControl::adjustAngle(double angle)
{
    std_msgs::Float64 msg;
    msg.data = (angle * -MAX_ANGLE + 1) / 2;
    out_angle.publish(msg);
}

void quit(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_control");
    RemoteControl remote_control;

    std::cout << "listining to keyboard and controller" << std::endl;
    std::cout << "====================================" << std::endl;

    signal(SIGINT, quit);
    remote_control.keyLoop();
    return 0;

    // ros::spin();
}
