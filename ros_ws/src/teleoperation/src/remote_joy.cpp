#include "remote_joy.h"

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 */
RemoteJoy::RemoteJoy()
{
    this->driveParametersPublisher =
        this->nodeHandle.advertise< drive_msgs::drive_param >(TOPIC_DRIVE_PARAMETERS, 1);

    this->joystickSubscriber = this->nodeHandle.subscribe< sensor_msgs::Joy >("joy", 10, &RemoteJoy::joystickCallback, this);
}

/**
 * @brief Callback function that is called each time a connected gamepad gets
 * an input
 *
 * @param joystick The data structure that contains information about the state of
 * the buttons and axes on the gamepad
 */
void RemoteJoy::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
    double steeringAngle = -joystick->axes[ JOYSTICK_AXIS_STEERING ];
    double velocity = (joystick->axes[ JOYSTICK_AXIS_THROTTLE ] - 1) * -0.5;
    bool deadMansSwitch = joystick->buttons[ JOYSTICK_BUTTON_DEADMANSSWITCH ] == 1;
    // TODO use deadMansSwitch

    publishDriveParameters(velocity, steeringAngle);
}

/**
 * @brief Publishes speed and angle values
 *
 * @param throttle The throttle provided by the gamepad input
 * @param steeringAngle The steering angle provided by the gamepad input
 */
void RemoteJoy::publishDriveParameters(double velocity, double steeringAngle)
{
    drive_msgs::drive_param driveParameters;
    driveParameters.velocity = velocity;
    driveParameters.angle = steeringAngle;

    this->driveParametersPublisher.publish(driveParameters);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "remote_joystick_controller");
    RemoteJoy remoteJoy;

    ros::spin();
}
