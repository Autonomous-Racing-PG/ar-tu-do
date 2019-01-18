#include "joystick_map.h"

class JoystickMapXboxone : public JoystickMap
{
    public:
    JoystickMapXboxone()
    {
    }

    ~JoystickMapXboxone()
    {
    }

    float getSteeringAxis(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return joystick->axes[0] * -1.0f;
    }

    float getAcceleration(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return (joystick->axes[5] - 1) * -0.5f;
    }

    float getDeceleration(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return (joystick->axes[2] - 1) * -0.5f;
    }

    bool isDeadMansSwitchPressed(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return joystick->buttons[0] == 1;
    }
};
