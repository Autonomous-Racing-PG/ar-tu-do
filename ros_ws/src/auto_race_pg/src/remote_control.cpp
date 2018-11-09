#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <termios.h>
#include <signal.h>

#define KEYCODE_W 119
#define KEYCODE_A 97
#define KEYCODE_S 115
#define KEYCODE_D 100

class RemoteControl
{
    public:
    RemoteControl();
    void keyLoop();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    int getch();

    void adjustCar(float speed, float angle);

    ros::NodeHandle nh_;

    int             linear_, angular_;
    double          l_scale_, a_scale_;

    ros::Publisher  out_ros;
    ros::Subscriber in_joy;
};

RemoteControl::RemoteControl()
    : linear_(1)
    , angular_(0)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    out_ros = nh_.advertise< geometry_msgs::Twist >("turtle1/cmd_vel", 1);

    in_joy =
        nh_.subscribe< sensor_msgs::Joy >("joy", 10, &RemoteControl::joyCallback, this);
}

void RemoteControl::keyLoop() {
    std::cout << "listening to keyboard and controller" << std::endl;
    std::cout << "====================================" << std::endl;
    while (ros::ok())
    {
	int c = getch();
        std::cout << c << std::endl;
        adjustCar(0, 0);
	
	float speed = 0;
	float angle = 0;

	switch(c) {
		case KEYCODE_W:
			speed++;
			break;
		case KEYCODE_S:
			speed--;
			break;
		case KEYCODE_A:
			angle--;
			break;
		case KEYCODE_D:
			angle++;
			break;
		default:
			break;
	}
	adjustCar(speed, angle);
    }
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    float speed = joy->axes[ linear_ ];
    float angle = joy->axes[ angular_ ];
    adjustCar(speed, angle);

    //geometry_msgs::Twist twist;
    //twist.angular.z = a_scale_ * joy->axes[ angular_ ];
    //twist.linear.x  = l_scale_ * joy->axes[ linear_ ];
    //ros_out.publish(twist);
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

void RemoteControl::adjustCar(float speed, float angle) {
    std::cout << speed << "  " << angle << std::endl;
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * angle;
    twist.linear.x = l_scale_ * speed;
    out_ros.publish(twist);
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

    //std::cout << "listining to keyboard and controller" << std::endl;
    //std::cout << "====================================" << std::endl;

    //signal(SIGINT, quit);
    //remote_control.keyLoop();
    //return 0;

    ros::spin();
}
