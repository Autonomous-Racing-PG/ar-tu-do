#pragma once

#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>

class VESCSimulator
{
    public:
    VESCSimulator();
    inline void setServoAngle(const double& data)
    {
        this->m_servo_data = data;
    }
    inline void setSpeed(const double& data)
    {
        this->m_state_speed = data;
    }
    void start();
    void stop();

    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_odometry_publisher;
    tf::TransformBroadcaster m_tf_publisher;
    ros::Timer m_timer;
    ros::Time m_last_stamp;

    bool m_started;
    double m_yaw;
    double m_servo_data;
    double m_state_speed;
    double m_x_position;
    double m_y_position;
    double m_x_dot;
    double m_y_dot;
    double m_current_speed;
    double m_current_steering_angle;
    double m_current_angular_velocity;
    double m_frequency;
    std::string m_odom_frame;
    std::string m_base_frame;

    void timerCallback(const ros::TimerEvent& event);
};