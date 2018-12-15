#include "joystick_controller.h"

/**
 * @brief Construct a new Remote Joy:: Remote Joy object.
 * It creates an subscriber for the joy topic.
 * It also creates 2 publishers that publish to the drive_parameters and dead_mans_switch topic
 */
JoystickController::JoystickController()
{
    this->m_drive_parameter_publisher = this->node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
    this->m_dead_mans_switch_publisher = this->node_handle.advertise<std_msgs::Int64>(TOPIC_DEAD_MANS_SWITCH, 1);

    this->m_joystick_subscriber =
        this->node_handle.subscribe<sensor_msgs::Joy>(TOPIC_JOY, 10, &JoystickController::joystickCallback, this);

    // The joystick_connection_timer gets started here with parameter oneshot = true. It will be started again inside
    // the callback. The timers sleep function is not used, because it can introduce a delay
    this->m_joystick_connection_timer =
        this->node_handle.createTimer(ros::Duration(0.1), &JoystickController::joystickConnectionCallback, this, true);
}

/**
 * @brief Callback function that is called each time a connected gamepad gets an input.
 * It publishes to the drive_parameters and dead_mans_switch topic.
 *
 * @param joystick The data structure that contains information about the state of the buttons and axes on the gamepad.
 */
void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
    // button value is 0 if not pressed and 1 if pressed
    int dms_button_value = joystick->buttons[JOYSTICK_BUTTON_DEADMANSSWITCH];
    ROS_ASSERT_MSG(dms_button_value != 0 && dms_button_value != 1, "Button value should be eather 0 or 1");

    // if button is pressed (and joystick is actually connected), send dead_mans_switch message
    if (dms_button_value == 1 && this->m_joystick_connected)
    {
        struct timeval time_struct;
        gettimeofday(&time_struct, NULL);
        long int timestamp = time_struct.tv_sec * 1000 + time_struct.tv_usec / 1000;
        std_msgs::Int64 dead_mans_switch_message;
        dead_mans_switch_message.data = timestamp;

        this->m_dead_mans_switch_publisher.publish(dead_mans_switch_message);
    }

    // compute and publish angle and velocity
    double steering_angle = joystick->axes[JOYSTICK_AXIS_STEERING] * -1.0f;
    double velocity = (joystick->axes[JOYSTICK_AXIS_THROTTLE] - 1) * -0.5f;

    this->publishDriveParameters(velocity, steering_angle);
}

/**
 * @brief Publishes speed and angle values
 *
 * @param velocity The velocity provided by the gamepad input
 * @param steering_angle The steering angle provided by the gamepad input
 */
void JoystickController::publishDriveParameters(double velocity, double steering_angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = steering_angle;

    this->m_drive_parameter_publisher.publish(drive_parameters);
}

void JoystickController::joystickConnectionCallback(const ros::TimerEvent&)
{
    // check if joystick is connected and assign result to m_joystick_connected
    std::string joystick_path = "/dev/input/js0";
    this->m_joystick_connected = (access(joystick_path.c_str(), F_OK) != -1);

    this->m_joystick_connection_timer =
        this->node_handle.createTimer(ros::Duration(0.1), &JoystickController::joystickConnectionCallback, this, true);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_controller");
    JoystickController joystick_controller;

    ros::spin();

    return EXIT_SUCCESS;
}
