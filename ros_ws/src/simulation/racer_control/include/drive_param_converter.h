#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>

/**
 * @brief Class to convert Drive Parameter Messages into single messages
 *
 * Class to convert Drive Parameter Messages (steering angle and velocity)
 * into single messages for each wheel velocity and for front wheel steering angles
 * based on Ackermann equations.
 */
class DriveParamConverter
{
    public:
    /**
     * @brief Constructor creates subscriber and publisher
     *
     */
    DriveParamConverter();

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param parameters contains steering angle for center of front axis and vehicle velocity
     */
    void convertDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    private:
    /**
     * @brief Return type to return two angles as return value
     *
     */
    struct AckermannSteeringAngles
    {
        double left_wheel_angle;
        double right_wheel_angle;
    };

    /**
     * @brief Calculates the angles of the front wheels based on the angle of the center of the front axis with
     * Ackermann equation/trigonometry
     *
     * @param angle Angle of the center of the front axis
     * @return Angles One Ackermann angle for each front wheel
     */
    AckermannSteeringAngles calculateSteeringAngles(const double& angle);

    /**
     * @brief ROS Handle
     *
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief ROS Subscriber for drive param messages
     *
     */
    ros::Subscriber m_drive_parameters_subscriber;

    /**
     * @brief ROS Publischer for the velocity of left rear wheel
     *
     */
    ros::Publisher m_left_rear_wheel_velocity_publisher;

    /**
     * @brief ROS Publischer for the velocity of right rear wheel
     *
     */
    ros::Publisher m_right_rear_wheel_velocity_publisher;
    /**
     * @brief ROS Publischer for the velocity of left front wheel
     *
     */
    ros::Publisher m_left_front_wheel_velocity_publisher;

    /**
     * @brief ROS Publischer for the velocity of right front wheel
     *
     */
    ros::Publisher m_right_front_wheel_velocity_publisher;

    /**
     * @brief ROS Publisher for the steering angle of left front wheel
     *
     */
    ros::Publisher m_left_steering_position_publisher;

    /**
     * @brief ROS Publisher for the steering angle of right front wheel
     *
     */
    ros::Publisher m_right_steering_position_publisher;
};