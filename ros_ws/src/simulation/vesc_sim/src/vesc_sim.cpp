#include "vesc_sim.h"
#include "car_config.h"
#include <nav_msgs/Odometry.h>

VESCSim::VESCSim()
{
    m_yaw = 0;
    m_servo_data = 0.5;
    m_state_speed = 0;
    m_x_position = 0;
    m_x_dot = 0;
    m_y_position = 0;
    m_y_dot = 0;
    m_current_speed = 0;
    m_current_steering_angle = 0;
    m_current_angular_velocity = 0;
    m_started = false;
    m_frequency = 50.0;
    m_last_stamp = ros::Time::now();
    m_odom_frame = "odom";
    m_base_frame = "base_link";

    this->m_odometry_publisher = this->m_node_handle.advertise<nav_msgs::Odometry>("odom", 10);
}

void VESCSim::start()
{
    if (!m_started)
    {
        m_timer = m_node_handle.createTimer(ros::Duration(1.0 / m_frequency), &VESCSim::timerCallback, this);
        m_started = true;
    }
}

void VESCSim::stop()
{
    if (m_started)
    {
        m_timer.stop();
        m_started = false;
    }
}

void VESCSim::timerCallback(const ros::TimerEvent& event)
{

    // convert to engineering units
    m_current_speed = (m_state_speed - car_config::SPEED_TO_ERPM_OFFSET) * car_config::ERPM_TO_SPEED;
    m_current_steering_angle =
        (m_servo_data - car_config::STEERING_TO_SERVO_OFFSET) / car_config::STEERING_TO_SERVO_GAIN;
    m_current_angular_velocity = m_current_speed * tan(m_current_steering_angle) / car_config::WHEELBASE;

    // calc elapsed time
    ros::Time stamp_now = ros::Time::now();
    ros::Duration dt = stamp_now - m_last_stamp;
    m_last_stamp = stamp_now;

    /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

    // propigate odometry
    m_x_dot = m_current_speed * cos(m_yaw);
    m_y_dot = m_current_speed * sin(m_yaw);
    m_x_position += m_x_dot * dt.toSec();
    m_y_position += m_y_dot * dt.toSec();
    m_yaw += m_current_angular_velocity * dt.toSec();

    // publish odometry message
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header.frame_id = m_odom_frame;
    odom->header.stamp = stamp_now;
    odom->child_frame_id = m_base_frame;

    // Position
    odom->pose.pose.position.x = m_x_position;
    odom->pose.pose.position.y = m_x_position;
    odom->pose.pose.orientation.x = 0.0;
    odom->pose.pose.orientation.y = 0.0;
    odom->pose.pose.orientation.z = sin(m_yaw / 2.0);
    odom->pose.pose.orientation.w = cos(m_yaw / 2.0);

    // Position uncertainty
    /** @todo Think about position uncertainty, perhaps get from parameters? */
    odom->pose.covariance[0] = 0.2;  ///< x
    odom->pose.covariance[7] = 0.2;  ///< y
    odom->pose.covariance[35] = 0.4; ///< yaw

    // Velocity ("in the coordinate frame given by the child_frame_id")
    odom->twist.twist.linear.x = m_current_speed;
    odom->twist.twist.linear.y = 0.0;
    odom->twist.twist.angular.z = m_current_angular_velocity;

    if (ros::ok())
    {
        m_odometry_publisher.publish(odom);
    }
}