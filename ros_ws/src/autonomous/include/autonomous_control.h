#include <cmath>
#include <cstdlib>
#include <iostream>

#include "drive_msgs/drive_param.h"
#include "drive_msgs/pid_input.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>

constexpr const char* TOPIC_DRIVE_PARAMETERS = "/input/drive_param/wallfollowing";

class AutonomousControl
{
    public:
    AutonomousControl();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_pid_input;

    /**
     * @brief The callback method for this node. Publishes the corrected values from
     * the wall following node to the car.
     *
     * @param pid_input
     */
    void pid_callback(const drive_msgs::pid_input::ConstPtr& pid_input);

    ros::Publisher drive_param_publisher;
};
