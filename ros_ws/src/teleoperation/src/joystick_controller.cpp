#include "joystick_controller.h"
#include "joystick_map_xbox360.h"
#include "joystick_map_ps3.h"

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


    // get the provided gamepad type
    ros::NodeHandle private_node_handle("~");
    private_node_handle.getParam(GAMEPAD_TYPE_PARAMETER, this->m_gamepad_type);

    // load joystick map for the provided gamepad type
    ROS_ASSERT_MSG(m_gamepad_type != "xbox360" || m_gamepad_type != "ps3", "Gamepad type should be xbox360 or ps3");
    if (m_gamepad_type == "xbox360")
    {
        m_joystick_map = new JoystickMapXbox360();
    }
    else if (m_gamepad_type == "ps3")
    {
        m_joystick_map = new JoystickMapPs3();
    }
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
    if (m_joystick_map->isDeadMansSwitchPressed(joystick))
    {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
        auto time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

        std_msgs::Int64 dead_mans_switch_message;
        dead_mans_switch_message.data = time_since_epoch.count();

        this->m_dms_publisher.publish(dead_mans_switch_message);
    }

    // compute and publish the provided steering and velocity
    float acceleration = m_joystick_map->getAcceleration(joystick) * ACCELERATION_SCALING_FACTOR;
    float deceleration = m_joystick_map->getDeceleration(joystick) * DECELERATION_SCALING_FACTOR;
    float steering_angle = m_joystick_map->getSteeringAxis(joystick) * STEERING_SCALING_FACTOR;

    this->publishDriveParameters(acceleration - deceleration, steering_angle);
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
