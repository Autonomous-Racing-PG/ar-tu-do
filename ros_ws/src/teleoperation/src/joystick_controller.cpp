#include "joystick_controller.h"

#include <std_msgs/Int64.h>

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 */
JoystickController::JoystickController()
{
    this->m_drive_parameter_publisher = this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
    this->dms_publisher = this->m_node_handle.advertise<std_msgs::Int64>(TOPIC_DMS, 1);

    this->m_joystick_subscriber =
        this->m_node_handle.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickController::joystickCallback, this);

    ros::NodeHandle private_node_handle("~");
    private_node_handle.getParam(INVERT_STEERING_PARAMETER, this->m_invert_steering);
}

/**
 * @brief Callback function that is called each time a connected gamepad gets
 * an input. It publishes a dms_message and drive_parameters.
 *
 * @param joystick The data structure that contains information about the state
 * of the buttons and axes on the gamepad
 */
void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
    // check if dms button is pressed. if yes -> send dms_message
    int dms_button_value = joystick->buttons[JOYSTICK_BUTTON_DEADMANSSWITCH];
    if (dms_button_value == 1) // 1 if button is pressed
    {
        struct timeval time_struct;
        gettimeofday(&time_struct, NULL);
        long int timestamp = time_struct.tv_sec * 1000 + time_struct.tv_usec / 1000;
        std_msgs::Int64 dms_message;
        dms_message.data = timestamp;

        this->dms_publisher.publish(dms_message);
    }

    // compute and publish angle and velocity
    double steering_angle = joystick->axes[JOYSTICK_AXIS_STEERING] * -1.0f;
    double velocity = (joystick->axes[JOYSTICK_AXIS_THROTTLE] - 1) * -0.5f;

    this->publishDriveParameters(velocity, steering_angle);

    // Detect if the button was pressed since the last reading
    bool invert_toggle_button = joystick->buttons[JOYSTICK_BUTTON_TOGGLE_INVERT_STEERING] == 1;
    if (invert_toggle_button && !this->m_toggle_invert_steering_state) {
        this->m_invert_steering = !this->m_invert_steering;
    }
    this->m_toggle_invert_steering_state = invert_toggle_button;
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

    this->m_drive_parameter_publisher.publish(drive_parameters);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_controller");
    JoystickController joystick_controller;

    ros::spin();

    return EXIT_SUCCESS;
}
