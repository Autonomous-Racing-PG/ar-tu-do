#include <ros/ros.h>

#include <time.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
#define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/angle"

#define TOPIC_STATUS_MODE "/status/mode"
#define TOPIC_STATUS_DMS "/status/dms"

#define TIMER_DURATION 150 // ms
#define DMS_MAX 200        // ms

#define MAX_SPEED 2000
#define MAX_ANGLE 0.8

class CarControl
{
    public:
    CarControl();

    private:
    ros::NodeHandle nh_;

    ros::Subscriber in_speed;
    ros::Subscriber in_angle;
    ros::Subscriber in_dms;

    void speed_callback(const std_msgs::Float64::ConstPtr& speed);
    void angle_callback(const std_msgs::Float64::ConstPtr& angle);
    void dms_callback(const std_msgs::Int64::ConstPtr& timestamp);

    ros::Publisher out_speed;
    ros::Publisher out_angle;
    ros::Publisher out_mode;

    void setMode(std::string mode);

    void adjustSpeed(double speed);
    void adjustAngle(double angle);

    bool run;
};

long dms = 0;

CarControl::CarControl()
    : run(true)
{
    in_speed =
        nh_.subscribe< std_msgs::Float64 >(TOPIC_SPEED, 1,
                                           &CarControl::speed_callback, this);
    in_angle =
        nh_.subscribe< std_msgs::Float64 >(TOPIC_ANGLE, 1,
                                           &CarControl::angle_callback, this);
    in_dms = nh_.subscribe< std_msgs::Int64 >(TOPIC_STATUS_DMS, 1,
                                              &CarControl::dms_callback, this);

    out_speed = nh_.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_SPEED, 1);
    out_angle = nh_.advertise< std_msgs::Float64 >(TOPIC_FOCBOX_ANGLE, 1);
    out_mode  = nh_.advertise< std_msgs::String >(TOPIC_STATUS_MODE, 1);
}

void CarControl::speed_callback(const std_msgs::Float64::ConstPtr& speed)
{
    adjustSpeed(speed->data);
}

void CarControl::angle_callback(const std_msgs::Float64::ConstPtr& angle)
{
    adjustAngle(angle->data);
}

void CarControl::dms_callback(const std_msgs::Int64::ConstPtr& timestamp)
{
    dms = timestamp->data;
}

void CarControl::adjustSpeed(double speed)
{
    if (!run)
    {
    }
    else
    {
        // std::cout << "speed: " << speed << std::endl;
        std_msgs::Float64 msg;
        msg.data = speed * MAX_SPEED;
        out_speed.publish(msg);
    }
}

void CarControl::setMode(std::string mode)
{
    std_msgs::String msg;
    msg.data = mode;
}

void CarControl::adjustAngle(double angle)
{
    if (!run)
    {
    }
    else
    {
        double            a = 0;
        double            b = 1;
        double            c = (1 - MAX_ANGLE);
        double            d = MAX_ANGLE;
        std_msgs::Float64 msg;
        msg.data = ((d - b) * angle + (b * c - a * d)) / (c - a);
        out_angle.publish(msg);
    }
}

void dms_timer_callback(const ros::TimerEvent& event)
{
    long now  = (long)(ros::Time::now().toSec() * 1000);
    long time = now - dms;
    if (time > DMS_MAX)
    {
        // std::cout << "fail-save active" << std::endl;
    }
    else
    {
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_control");
    CarControl      car_control;
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(TIMER_DURATION / 1000.0),
                                      dms_timer_callback);

    ros::spin();
    return 0;
}
