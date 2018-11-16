#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#define TOPIC_FOCBOX_SPEED "/commands/motor/speed"
#define TOPIC_FOCBOX_ANGLE "/commands/servo/position"

#define TOPIC_SPEED "/set/speed"
#define TOPIC_ANGLE "/set/angle"

#define TOPIC_STATUS_MODE "/status/mode"
#define TOPIC_STATUS_DEAD_MANS_SWITCH "/status/deadmansswitch"

#define TIMER_DURATION 250 // ms

#define MAX_SPEED 5000
#define MAX_ANGLE 0.8


class CarControl
{
    public:
    CarControl();

    private:
    ros::NodeHandle nh_;

    ros::Subscriber  in_speed;
    ros::Subscriber  in_angle;
    ros::Subscriber  in_dead_mans_switch;

    void speed_callback(const std_msgs::Float64::ConstPtr &speed);
    void angle_callback(const std_msgs::Float64::ConstPtr &angle);
    void dms_callback(const std_msgs::Empty::ConstPtr &empty);

    ros::Publisher  out_speed;
    ros::Publisher  out_angle;
    ros::Publisher  out_mode;

    void setMode(std::string mode);

    void adjustSpeed(double speed);
    void adjustAngle(double angle);

    bool run;
};

bool dms = true;
int dead_countdown;

CarControl::CarControl()
: run(true)
{
    in_speed =
    nh_.subscribe < std_msgs::Float64 > (TOPIC_SPEED, 1, & CarControl::speed_callback, this);
    in_angle =
    nh_.subscribe < std_msgs::Float64 > (TOPIC_ANGLE, 1, & CarControl::angle_callback, this);
    in_dead_mans_switch =
    nh_.subscribe < std_msgs::Empty > (TOPIC_STATUS_DEAD_MANS_SWITCH, 1, & CarControl::dms_callback, this);

    out_speed = nh_.advertise < std_msgs::Float64 > (TOPIC_FOCBOX_SPEED, 1);
    out_angle = nh_.advertise < std_msgs::Float64 > (TOPIC_FOCBOX_ANGLE, 1);
    out_mode = nh_.advertise < std_msgs::String > (TOPIC_STATUS_MODE, 1);
}


void CarControl::speed_callback(const std_msgs::Float64::ConstPtr & speed) {
    adjustSpeed(speed -> data);
}

void CarControl::angle_callback(const std_msgs::Float64::ConstPtr & angle) {
    adjustAngle(angle -> data);
}

void CarControl::dms_callback(const std_msgs::Empty::ConstPtr & e) {
    dms = true;
}

void CarControl::adjustSpeed(double speed) {
    if(!run) {

    }
    else {
        std::cout << "speed: " << speed << std::endl;
        std_msgs::Float64 msg;
        msg.data = speed * MAX_SPEED;
        out_speed.publish(msg);
    }
}

void CarControl::setMode(std::string mode) {
    std_msgs::String msg;
    msg.data = mode;
}

void CarControl::adjustAngle(double angle) {
    if(!run) {

    }
    else {
        std::cout << "angle: " << angle << std::endl;
        std_msgs::Float64 msg;
        msg.data = (-angle * MAX_ANGLE + 1) / 2;
        out_angle.publish(msg);
    }
}

void dms_timer_callback(const ros::TimerEvent &event) {
    if(!dms) {
        std::cout << "fail-save active" << std::endl;
    } else {
        dms = false;
    }
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "car_control");
    CarControl car_control;
    ros::NodeHandle nh;
    //ros::Timer timer = nh.createTimer(ros::Duration(TIMER_DURATION / 1000.0), dms_timer_callback); 

    ros::spin();
    return 0;
}
