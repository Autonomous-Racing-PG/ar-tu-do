# include <ros/ros.h>

# include <std_msgs/Float64.h>

# define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
# define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

# define TOPIC_SPEED "/set/speed"
# define TOPIC_ANGLE "/set/position"

# define MAX_SPEED 5000
# define MAX_ANGLE 0.8


class CarControl
{
    public:
    CarControl();

    private:
    ros::NodeHandle nh_;

    ros::Subscriber  in_speed;
    ros::Subscriber  in_angle;

    void speed_callback(const std_msgs::Float64::ConstPtr & speed);
    void angle_callback(const std_msgs::Float64::ConstPtr & angle);

    ros::Publisher  out_speed;
    ros::Publisher  out_angle;

    void adjustSpeed(double speed);
    void adjustAngle(double angle);

    bool run;
};

CarControl::CarControl()
: run(true)
{
    out_speed = nh_.advertise < std_msgs::Float64 > (TOPIC_FOCBOX_SPEED, 1);
    out_angle = nh_.advertise < std_msgs::Float64 > (TOPIC_FOCBOX_ANGLE, 1);

    in_speed =
    nh_.subscribe < std_msgs:: Float64 > (TOPIC_SPEED, 1, & CarControl::speed_callback, this);
    in_angle =
    nh_.subscribe < std_msgs:: Float64 > (TOPIC_ANGLE, 1, & CarControl::angle_callback, this);
}

void CarControl::speed_callback(const std_msgs::Float64::ConstPtr & speed) {
    adjustSpeed(speed -> data);
}

void CarControl::angle_callback(const std_msgs::Float64::ConstPtr & angle) {
    adjustAngle(angle -> data);
}


void CarControl:: adjustSpeed(double speed) {
    if(!run) {
        std::cout << "fail safe" << std::endl;
    }
    else {
        std::cout << "speed: " << speed << std::endl;
        std_msgs::Float64 msg;
        msg.data = speed * MAX_SPEED;
        out_speed.publish(msg);
    }
}

void CarControl::adjustAngle(double angle) {
    if(!run) {
        std::cout << "fail safe" << std::endl;
    }
    else {
        std::cout << "angle: " << angle << std::endl;
        std_msgs::Float64 msg;
        msg.data = (-angle * MAX_ANGLE + 1) / 2;
        out_angle.publish(msg);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "car_control");
    CarControl car_control;

    ros::spin();
}
