#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <termios.h>
#include <signal.h>

#define KEYCODE_A 97; 
#define KEYCODE_S 115; 
#define KEYCODE_D 100; 
#define KEYCODE_W 119; 

#define KEYCODE_A 97; 
#define KEYCODE_A 97; 
#define KEYCODE_A 97; 
#define KEYCODE_A 97; 

class RemoteControl
{
    public:
    RemoteControl();
    void keyLoop();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void testCallback(const geometry_msgs::Twist::ConstPtr& twist);

    int getch();

    void adjustCar(int angle, int speed);

    ros::NodeHandle nh_;

    int             linear_, angular_;
    double          l_scale_, a_scale_;
    ros::Publisher  ros_out;
    ros::Subscriber joy_in;
    ros::Subscriber ros_in;
};

RemoteControl::RemoteControl()
    : linear_(1)
    , angular_(2)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    ros_out = nh_.advertise< geometry_msgs::Twist >("turtle1/cmd_vel", 1);

    joy_in =
        nh_.subscribe< sensor_msgs::Joy >("joy", 10,
                                          &RemoteControl::joyCallback, this);

    ros_in =
        nh_.subscribe< geometry_msgs::Twist >("turtle1/cmd_vel", 1,
                                            &RemoteControl::testCallback, this);
}

void RemoteControl::keyLoop() {
    std::cout << "listening to keyboard" << std::endl;
    std::cout << "=====================" << std::endl;
    while (ros::ok())
    {
        int c = getch();   // call your non-blocking input function
        std::cout << c << std::endl;

        adjustCar(0, 0);
    }
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[ angular_ ];
    twist.linear.x  = l_scale_ * joy->axes[ linear_ ];
    ros_out.publish(twist);
}

void RemoteControl::testCallback(const geometry_msgs::Twist::ConstPtr& twist)
{

}

int RemoteControl::getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void RemoteControl::adjustCar(int angle, int speed) {

}

void quit(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_control");
    RemoteControl remove_control;

    signal(SIGINT, quit);
    remove_control.keyLoop();

    return 0;
}
