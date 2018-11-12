# include <ros/ros.h>

# include <std_msgs/Float64.h>
# include <sensor_msgs/Joy.h>

# define JOY_ANGLE_ANGULAR 0
# define JOY_ANGLE_LINEAR 1

# define TOPIC_SPEED "/set/speed"
# define TOPIC_ANGLE "/set/position"


class RemoteJoy
{
    public:
    RemoteJoy();
    void keyLoop();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);

    void adjustAngle(double angle);
    void adjustSpeed(double speed);

    ros::NodeHandle nh_;

    std::string input;

    ros::Publisher  out_speed;
    ros::Publisher  out_angle;

    ros::Subscriber in_joy;
};

RemoteJoy::RemoteJoy()
{
    out_speed = nh_.advertise < std_msgs::Float64 > (TOPIC_SPEED, 1);
    out_angle = nh_.advertise < std_msgs::Float64 > (TOPIC_ANGLE, 1);

    in_joy =
    nh_.subscribe < sensor_msgs::Joy > ("joy", 10, & RemoteJoy::joyCallback, this);
}

void RemoteJoy::joyCallback(const sensor_msgs::Joy::ConstPtr & joy)
{
    double angle = joy -> axes[JOY_ANGLE_ANGULAR];
    double speed = joy -> axes[JOY_ANGLE_LINEAR];
    adjustSpeed(speed);
    adjustAngle(angle);
}

void RemoteJoy::adjustAngle(double angle) {
    std_msgs::Float64 msg;
    msg.data = (angle + 1) / 2;
    out_angle.publish(msg);
}

void RemoteJoy::adjustSpeed(double speed) {
    std_msgs::Float64 msg;
    msg.data = speed;
    out_speed.publish(msg);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "remove_joy");
    RemoteJoy remote_joy;

    ros::spin();
}
