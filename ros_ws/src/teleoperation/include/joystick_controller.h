#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <drive_msgs/drive_param.h>
#include <sensor_msgs/Joy.h>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <teleoperation/joystick_controllerConfig.h>

constexpr const char* PARAMETER_JOYSTICK_TYPE = "joystick_type";
constexpr const char* TOPIC_DRIVE_PARAMETERS = "input/drive_param/joystick";
constexpr const char* TOPIC_HEARTBEAT_MANUAL = "/input/heartbeat_manual";
constexpr const char* TOPIC_HEARTBEAT_AUTONOMOUS = "/input/heartbeat_autonomous";

constexpr float EPSILON = 0.001;

class JoystickController
{

    /**
     * @brief scales the absolute acceleration provided by the joystick.
     * Useful if the car should not drive with 100% speed if acceleration button is fully pressed
     */
    float m_acceleration_scaling_factor = 0.35f;

    /**
     * @brief scales the absolute deceleration provided by the joystick.
     * Useful if the car should not decelerate with 100% speed if deceleration button is fully pressed
     */
    float m_deceleration_scaling_factor = 0.35f;

    /**
     * @brief scales the absolute steering value (between -1 and 1) provided by the joystick.
     * Useful if the car should not steer 100% left and right
     */
    float m_steering_scaling_factor = 0.8f;

    struct JoystickMapping
    {
        uint_fast16_t steeringAxis;
        uint_fast16_t accelerationAxis;
        uint_fast16_t decelerationAxis;
        uint_fast16_t enableManualButton;
        uint_fast16_t enableAutonomousButton;
    };

    const struct JoystickMapping joystick_mapping_ps3 = { 0, 13, 12, 14, 13 };
    const struct JoystickMapping joystick_mapping_xbox360 = { 0, 4, 5, 0, 1 };
    const struct JoystickMapping joystick_mapping_xboxone = { 0, 5, 2, 0, 1 };

    public:
    JoystickController();

    private:
    dynamic_reconfigure::Server<teleoperation::joystick_controllerConfig> m_dyn_cfg_server;

    ros::NodeHandle m_node_handle;
    ros::Publisher m_drive_parameter_publisher;
    ros::Subscriber m_joystick_subscriber;
    ros::Publisher m_enable_manual_publisher;
    ros::Publisher m_enable_autonomous_publisher;

    JoystickMapping m_joystick_map;

    /**
     * @brief Due to a bug with the joy node, joystick axes return non-zero values when the axis is at zero
     * until the axis is changed for the first time. Thus, this variable saves whether a zero was ever received.
     */
    bool m_acceleration_locked;
    bool m_deceleration_locked;

    /**
     * @brief Callback function that is called each time a connected gamepad gets an input. It publishes a dms_message
     * and drive_parameters.
     *
     * @param joystick The data structure that contains information about the state of the buttons and axes on the
     * gamepad
     */
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);

    /**
     * @brief Publishes speed and angle values
     *
     * @param velocity The velocity provided by the gamepad input
     * @param steering_angle The steering angle provided by the gamepad input
     */
    void publishDriveParameters(double velocity, double steering_angle);

    /**
     * @brief Looks for a provided joystick_type argument and selects the corresponding JoystickMapping
     */
    void selectJoystickMapping();

    /**
     * @brief The current value of the members (with regarding to the dyn config) are updated on the server
     */
    void updateDynamicConfig();
};
