#pragma once
#include <string>
#include <cmath>
#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>


/**
 * @brief 
 * 
 */
struct Angles
{
    double angle1;
    double angle2;
};

/**
 * @brief 
 * 
 */
class DriveParamConverter
{
    public:
    /**
     * @brief Construct a new Drive Pram Converter object
     * 
     */
    DriveParamConverter();
    /**
     * @brief 
     * 
     * @param parameters 
     */
    void convertDriveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);
    

    private:
    /**
     * @brief 
     * 
     * @param angle Angle of the center of the front axis.
     * @return Angles One Ackermann angle for each front wheel
     */
    Angles calculateSteeringAngles(const double& angle);
    /**
     * @brief 
     * 
     */
    ros::NodeHandle node_handle;
    /**
     * @brief 
     * 
     */
    ros::Subscriber m_drive_parameters_subscriber;
    /**
     * @brief 
     * 
     */
    ros::Publisher m_left_rear_wheel_velocity_publischer;
    /**
     * @brief 
     * 
     */
    ros::Publisher m_right_rear_wheel_velocity_publischer;
    /**
     * @brief 
     * 
     */
    ros::Publisher m_left_front_wheel_velocity_publischer;
    /**
     * @brief 
     * 
     */
    ros::Publisher m_right_front_wheel_velocity_publischer;
    /**
     * @brief 
     * 
     */
    ros::Publisher m_left_steering_position_publischer;
    /**
     * @brief 
     * 
     */
    ros::Publisher m_right_steering_position_publischer;
    
    /**
     * @brief 
     * 
     */
    double m_length;

    /**
     * @brief 
     * 
     */
    double m_width;

    std::string m_TOPIC_DRIVE_PARAMETERS;
};