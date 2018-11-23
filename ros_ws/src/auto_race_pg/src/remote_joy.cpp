#include "remote_joy.h"

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 *
 */
RemoteJoy::RemoteJoy()
{
    out_speed = nh_.advertise< std_msgs::Float64 >(TOPIC_SPEED, 1);
    out_angle = nh_.advertise< std_msgs::Float64 >(TOPIC_ANGLE, 1);

    in_joy = nh_.subscribe< sensor_msgs::Joy >("joy", 10,
                                               &RemoteJoy::joyCallback, this);
}

/**
 * @brief Callback function that gets called each time a connected gamepad get
 * an input
 *
 * @param joy The data structure that contains informations about the state of
 * each avaliable key on the gamepad
 */
void RemoteJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double angle = joy->axes[ JOY_ANGLE_ANGULAR ] * (-1);
    double speed = (joy->axes[ 5 ] - 1) * (-0.5);
    bool   dms   = joy->buttons[ 0 ] == 1;
    std::cout << dms << std::endl;
    publishSpeed(speed);
    publishAngle(angle);
}

/**
 * @brief Converts the given angle to the right range and publishes it
 *
 * @param angle The angle provided by the gamepad input
 */
void RemoteJoy::publishAngle(double angle)
{
    std_msgs::Float64 msg;
    msg.data = angle;
    out_angle.publish(msg);
}

/**
 * @brief Converts the given speed to the right range and publishes it
 *
 * @param speed The speed provided by the gamepad input
 */
void RemoteJoy::publishSpeed(double speed)
{
    std_msgs::Float64 msg;
    msg.data = speed;
    out_speed.publish(msg);
}

/**
 * @brief
 *
 * @param is_pressed
 */
// void RemoteJoy::publishDeadManSwitch(bool is_pressed)
// {
//     std_msgs::Int64 msg;
//     msg.data = (long)(ros::Time::now().toSec() * 1000);
//     out_dms.publish(msg);
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_joy");
    RemoteJoy remote_joy;

    ros::spin();
}
