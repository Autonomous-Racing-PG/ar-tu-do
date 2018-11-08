#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class RemoteControl
{
    public:
    RemoteControl();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void testCallback(const geometry_msgs::Twist::ConstPtr& twist);

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_control");
    RemoteControl remove_control;

    ros::spin();
}
