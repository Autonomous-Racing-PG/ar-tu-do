#include "remote_joy.h"

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 *
 */
RemoteJoy::RemoteJoy()
{
    out_drive_param =
        nh_.advertise< drive_msgs::drive_param >(TOPIC_DRIVE_PARAM, 1);

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
    publishDriveParam(speed, angle);
}

/**
 * @brief Converts the given speed and angle to the right range and publishes it
 *
 * @param speed The speed provided by the gamepad input
 * @param angle The angle provided by the gamepad input
 */
void RemoteJoy::publishDriveParam(double speed, double angle)
{
    drive_msgs::drive_param msg;
    msg.velocity = speed;
    msg.angle    = angle;
    out_drive_param.publish(msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_joy");
    RemoteJoy remote_joy;

    ros::spin();
}
