#include "joystick_map.h"

class JoystickMapPs3 : public JoystickMap
{
    public:
    JoystickMapPs3()
    {
    }

    ~JoystickMapPs3()
    {
    }

    float getSteeringAxis(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return joystick->axes[0] * -1.0f;
    }

    float getAcceleration(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return (joystick->axes[13] - 1) * -0.5f;
    }

    float getDeceleration(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return (joystick->axes[12] - 1) * -0.5f;
    }

    bool isDeadMansSwitchPressed(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return joystick->buttons[14] == 1;
    }
};
