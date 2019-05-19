#include "joystick_controller.h"

#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Time.h>

#include <teleoperation/joystick_controllerConfig.h>

/**
 * @brief Construct a new Remote Joy:: Remote Joy object
 */
JoystickController::JoystickController()
{
    this->m_drive_parameter_publisher =
        this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
    this->m_enable_manual_publisher = this->m_node_handle.advertise<std_msgs::Time>(TOPIC_HEARTBEAT_MANUAL, 1);
    this->m_enable_autonomous_publisher = this->m_node_handle.advertise<std_msgs::Time>(TOPIC_HEARTBEAT_AUTONOMOUS, 1);

    this->m_joystick_subscriber =
        this->m_node_handle.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickController::joystickCallback, this);

    this->selectJoystickMapping();
    this->m_acceleration_locked = true;
    this->m_deceleration_locked = true;

    this->updateDynamicConfig();
    m_dyn_cfg_server.setCallback([&](teleoperation::joystick_controllerConfig& cfg, uint32_t) {
        m_joystick_map.steeringAxis = cfg.joystick_steering_axis;
        m_joystick_map.accelerationAxis = cfg.joystick_acceleration_axis;
        m_joystick_map.decelerationAxis = cfg.joystick_deceleration_axis;
        m_joystick_map.enableManualButton = cfg.joystick_enable_manual_button;
        m_joystick_map.enableAutonomousButton = cfg.joystick_enable_autonomous_button;

        m_acceleration_scaling_factor = cfg.acceleration_scaling_factor;
        m_deceleration_scaling_factor = cfg.deceleration_scaling_factor;
        m_steering_scaling_factor = cfg.steering_scaling_factor;
    });
}

std_msgs::Time createHearbeatMessage()
{
    std_msgs::Time message;
    message.data = ros::Time::now();
    return message;
}

void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
    ROS_ASSERT_MSG(m_joystick_map.accelerationAxis < joystick->axes.size(),
                   "Invalid index access on joystick axis array");
    ROS_ASSERT_MSG(m_joystick_map.decelerationAxis < joystick->axes.size(),
                   "Invalid index access on joystick axis array");
    ROS_ASSERT_MSG(m_joystick_map.steeringAxis < joystick->axes.size(), "Invalid index access on joystick axis array");

    ROS_ASSERT_MSG(m_joystick_map.enableManualButton < joystick->buttons.size(),
                   "Invalid index access on joystick button array");
    ROS_ASSERT_MSG(m_joystick_map.enableAutonomousButton < joystick->buttons.size(),
                   "Invalid index access on joystick button array");

    if (joystick->buttons[m_joystick_map.enableManualButton] == 1)
    {
        this->m_enable_manual_publisher.publish(createHearbeatMessage());
    }
    if (joystick->buttons[m_joystick_map.enableAutonomousButton] == 1)
    {
        this->m_enable_autonomous_publisher.publish(createHearbeatMessage());
    }

    // compute and publish the provided steering and velocity
    float acceleration = (joystick->axes[m_joystick_map.accelerationAxis] - 1) * -0.5f * m_acceleration_scaling_factor;
    float deceleration = (joystick->axes[m_joystick_map.decelerationAxis] - 1) * -0.5f * m_deceleration_scaling_factor;

    if (this->m_acceleration_locked)
    {
        if (std::abs(acceleration) < EPSILON)
        {
            this->m_acceleration_locked = false;
        }
        else
        {
            acceleration = 0;
        }
    }
    if (this->m_deceleration_locked)
    {
        if (std::abs(deceleration) < EPSILON)
        {
            this->m_deceleration_locked = false;
        }
        else
        {
            deceleration = 0;
        }
    }

    float velocity = acceleration - deceleration;

    float steering_angle = joystick->axes[m_joystick_map.steeringAxis] * -1.0f * m_steering_scaling_factor;

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
        ROS_INFO_STREAM(PARAMETER_JOYSTICK_TYPE << " : " << joystick_type);
        m_joystick_map = joystick_mapping_xbox360;
    }
    this->updateDynamicConfig();
}

void JoystickController::updateDynamicConfig()
{
    teleoperation::joystick_controllerConfig cfg;
    {
        cfg.joystick_steering_axis = m_joystick_map.steeringAxis;
        cfg.joystick_acceleration_axis = m_joystick_map.accelerationAxis;
        cfg.joystick_deceleration_axis = m_joystick_map.decelerationAxis;
        cfg.joystick_enable_manual_button = m_joystick_map.enableManualButton;
        cfg.joystick_enable_autonomous_button = m_joystick_map.enableAutonomousButton;

        cfg.acceleration_scaling_factor = m_acceleration_scaling_factor;
        cfg.deceleration_scaling_factor = m_deceleration_scaling_factor;
        cfg.steering_scaling_factor = m_steering_scaling_factor;
    }
    m_dyn_cfg_server.updateConfig(cfg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_controller");
    JoystickController joystick_controller;

    ros::spin();

    return EXIT_SUCCESS;
}
