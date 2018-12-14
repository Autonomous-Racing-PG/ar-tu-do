#include "joystick_controller.h"

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 */
JoystickController::JoystickController()
{
    this->drive_parameter_publisher = this->node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);

    this->joystick_subscriber =
        this->node_handle.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickController::joystickCallback, this);
}

/**
 * @brief Callback function that is called each time a connected gamepad gets
 * an input
 *
 * @param joystick The data structure that contains information about the state
 * of
 * the buttons and axes on the gamepad
 */
void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
    double steering_angle = -joystick->axes[JOYSTICK_AXIS_STEERING];
    double velocity = (joystick->axes[JOYSTICK_AXIS_THROTTLE] - 1) * -0.5;

    this->publishDriveParameters(velocity, steering_angle);
}

/**
 * @brief Publishes speed and angle values
 *
 * @param throttle The throttle provided by the gamepad input
 * @param steering_angle The steering angle provided by the gamepad input
 */
void JoystickController::publishDriveParameters(double velocity, double steering_angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = steering_angle;

    this->drive_parameter_publisher.publish(drive_parameters);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_controller");
    JoystickController joystick_controller;

    ros::spin();

    return EXIT_SUCCESS;
}
