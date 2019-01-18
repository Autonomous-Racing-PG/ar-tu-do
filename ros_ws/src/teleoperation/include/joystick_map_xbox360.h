#include "joystick_map.h"

class JoystickMapXbox360 : public JoystickMap
{
    public:
    JoystickMapXbox360()
    {
    }

    ~JoystickMapXbox360()
    {
    }

    float getSteeringAxis(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return joystick->axes[0] * -1.0f;
    }

    float getAcceleration(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return (joystick->axes[4] - 1) * -0.5f;
    }

    float getDeceleration(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return (joystick->axes[5] - 1) * -0.5f;
    }

    bool isDeadMansSwitchPressed(const sensor_msgs::Joy::ConstPtr& joystick)
    {
        return joystick->buttons[0] == 1;
    }
};
