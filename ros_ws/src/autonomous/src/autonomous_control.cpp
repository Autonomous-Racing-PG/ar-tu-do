#include "autonomous_control.h"

AutonomousControl::AutonomousControl()
{
    m_pid_input =
        m_node_handle.subscribe<drive_msgs::pid_input>("/pid_input", 1, &AutonomousControl::pid_callback, this);
    drive_param_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS, 1);
}

void AutonomousControl::pid_callback(const drive_msgs::pid_input::ConstPtr& pid_input)
{

    float vel = pid_input->pid_vel;
    float error = pid_input->pid_error;

    // ROS_INFO_STREAM("vel: " << vel << std::endl);
    // ROS_INFO_STREAM("error: " << error << std::endl);

    drive_msgs::drive_param control;
    control.velocity = vel;
    control.angle = error;
    drive_param_publisher.publish(control);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    AutonomousControl autonomous_control;
    ros::spin();
    return EXIT_SUCCESS;
}