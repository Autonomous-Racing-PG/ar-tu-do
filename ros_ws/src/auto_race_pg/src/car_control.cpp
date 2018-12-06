#include "car_control.h"

CarControl::CarControl()
    : run{ true }
    , speed{ 0 }
    , angle{ 0 }
{
    in_speed =
        nh_.subscribe< std_msgs::Float64 >(TOPIC_SPEED, 1,
                                           &CarControl::speed_callback, this);
    in_angle =
        nh_.subscribe< std_msgs::Float64 >(TOPIC_ANGLE, 1,
                                           &CarControl::angle_callback, this);
    in_cmd =
        nh_.subscribe< std_msgs::String >(TOPIC_CMD, 1,
                                           &CarControl::cmd_callback, this);

    out_speed = nh_.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_SPEED, 1);
    out_angle = nh_.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_ANGLE, 1);
}

void CarControl::speed_callback(const std_msgs::Float64::ConstPtr& speed)
{
    adjustSpeed(speed->data);
}

void CarControl::angle_callback(const std_msgs::Float64::ConstPtr& angle)
{
    adjustAngle(angle->data);
}
void CarControl::cmd_callback(const std_msgs::String::ConstPtr& cmd)
{
    std::string str = cmd->data;
    if(str.compare("stop") == 0) {
        run = false;
        adjustSpeed(0);
        adjustAngle(0);
    }
    if(str.compare("go") == 0) {
        run = true;
    }
}

void CarControl::adjustSpeed(double raw)
{
    speed = raw * MAX_SPEED;
    if (speed < MIN_SPEED)
    {
        speed = 0;
    }
    std::cout << "speed: " << speed << " | angle: " << angle << std::endl;
    if (run)
    {
        std_msgs::Float64 msg;
        msg.data = speed;
        out_speed.publish(msg);
    }
}

void CarControl::adjustAngle(double raw)
{
    angle = (raw * MAX_ANGLE + 1) / 2;
    std::cout << "speed: " << speed << " | angle: " << angle << std::endl;
    if (run)
    {
        std_msgs::Float64 msg;
        msg.data = angle;
        out_angle.publish(msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_control");
    CarControl car_control;
    ros::spin();
    return 0;
}
