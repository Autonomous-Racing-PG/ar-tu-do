#include "joystick_controller.h"

#include <ros/console.h>
#include <std_msgs/Int64.h>

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 */
JoystickController::JoystickController()
{
    this->m_drive_parameter_publisher =
        this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
    this->m_dms_publisher = this->m_node_handle.advertise<std_msgs::Int64>(TOPIC_DMS, 1);

    this->m_joystick_subscriber =
        this->m_node_handle.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickController::joystickCallback, this);

    this->selectJoystickMapping();
}

void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
    // check if dms button is pressed. if yes -> send dms_message
    if (joystick->buttons[m_joystick_map.deadMansSwitchButton] == 1)
    {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
        auto time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

        std_msgs::Int64 dead_mans_switch_message;
        dead_mans_switch_message.data = time_since_epoch.count();

        this->m_dms_publisher.publish(dead_mans_switch_message);
    }

    // compute and publish the provided steering and velocity
    float acceleration = (joystick->axes[m_joystick_map.accelerationAxis] - 1) * -0.5f * ACCELERATION_SCALING_FACTOR;
    float deceleration = (joystick->axes[m_joystick_map.decelerationAxis] - 1) * -0.5f * DECELERATION_SCALING_FACTOR;
    float velocity = acceleration - deceleration;

    float steering_angle = joystick->axes[m_joystick_map.steeringAxis] * -1.0f * STEERING_SCALING_FACTOR;

    ROS_ASSERT_MSG(velocity >= -1.0f && velocity <= 1.0f, "Velocity should be between -1 and 1");
    ROS_ASSERT_MSG(steering_angle >= -1.0f && steering_angle <= 1.0f, "Steering angle should be between -1 and 1");

    this->publishDriveParameters(acceleration - deceleration, steering_angle);
}

void JoystickController::publishDriveParameters(double velocity, double steering_angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = steering_angle;

    this->m_drive_parameter_publisher.publish(drive_parameters);
}

void JoystickController::selectJoystickMapping()
{
    std::string joystick_type = "";
    ros::NodeHandle private_node_handle("~");
    private_node_handle.getParam(PARAMETER_JOYSTICK_TYPE, joystick_type);

    if (joystick_type == "xbox360")
    {
        m_joystick_map = joystick_mapping_xbox360;
    }
    else if (joystick_type == "ps3")
    {
        m_joystick_map = joystick_mapping_ps3;
    }
    else if (joystick_type == "xboxone")
    {
        m_joystick_map = joystick_mapping_xboxone;
    }
    else
    {
        ROS_WARN_STREAM("No valid joystick_type argument provided. Falling back to xbox360 keybindings");
        m_joystick_map = joystick_mapping_xbox360;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_controller");
    JoystickController joystick_controller;

    ros::spin();

    return EXIT_SUCCESS;
}
