#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

class JoystickMap
{
    protected:
    JoystickMap()
    {
    }

    public:
    virtual ~JoystickMap()
    {
    }

    /**
     * @brief Get value of the steering axis. -1 is maximal left, 1 is maximal right
     *
     * @return float
     */
    virtual float getSteeringAxis(const sensor_msgs::Joy::ConstPtr& joystick) = 0;

    /**
     * @brief Get value of the acceleration. 0 is no acceleration, 1 is maximal acceleration
     * 
     * @return float 
     */
    virtual float getAcceleration(const sensor_msgs::Joy::ConstPtr& joystick) = 0;

    /**
     * @brief Get value of the deceleration. 0 is no deceleration, 1 is maximal deceleration
     *
     * @return float
     */
    virtual float getDeceleration(const sensor_msgs::Joy::ConstPtr& joystick) = 0;

    /**
     * @brief Returns whether or not dead man's switch is pressed.
     *
     * @return true
     * @return false
     */
    virtual bool isDeadMansSwitchPressed(const sensor_msgs::Joy::ConstPtr& joystick) = 0;
};