#include "dms_controller.h"

#include "car_control.h"

/**
 * Class constructor that sets up a subscriber that listens for dms messages.
 * */
DMSController::DMSController()
{

    this->m_dms_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int64>(TOPIC_DMS, 1, &DMSController::dmsCallback, this);

    this->m_command_pulisher = this->m_node_handle.advertise<std_msgs::String>(TOPIC_COMMAND, 1);

    ros::NodeHandle private_node_handle("~");

    private_node_handle.getParam(PARAMETER_DMS_CHECK_RATE, this->dms_check_rate);
    if (dms_check_rate <= 0 || dms_check_rate > 1000)
    {
        ROS_WARN_STREAM("dms_check_rate should be bigger than 0 and smaller or equal to 1000. Your value: "
                        << dms_check_rate << ", new value: 20.");
        dms_check_rate = 20;
        
    }

    private_node_handle.getParam(PARAMETER_DMS_EXPIRATION, this->dms_expiration);
    if (dms_expiration <= 0 || dms_expiration > 1000)
    {
        ROS_WARN_STREAM("dms_expiration should be bigger than 0 and smaller or equal to 1000. Your value: "
                        << dms_expiration << ", new value: 100.");
        dms_expiration = 100;
    }
}

void DMSController::checkDMS()
{
    auto current_time = std::chrono::steady_clock::now();

    if (m_running)
    {
        // switch is triggered

        if (m_last_dms_message_received + DMS_EXPIRATION < current_time)
        {
            // switch is not triggered anymode
            m_running = false;
            std_msgs::String command_message;
            command_message.data = COMMAND_STOP;
            this->m_command_pulisher.publish(command_message);
        }
    }
    else
    {
        // switch is not triggered

        if (m_last_dms_message_received + DMS_EXPIRATION >= current_time)
        {

            // switch is is triggered again
            m_running = true;
            std_msgs::String command_message;
            command_message.data = COMMAND_GO;
            this->m_command_pulisher.publish(command_message);
        }
    }
}

void DMSController::dmsCallback(const std_msgs::Int64::ConstPtr& dms_message)
{
    std::chrono::milliseconds time_since_epoch(dms_message->data);
    this->m_last_dms_message_received = std::chrono::time_point<std::chrono::steady_clock>(time_since_epoch);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dms_controller");
    DMSController dmsController;

    ros::Rate loop_rate(dmsController.dms_check_rate);
    while (ros::ok())
    {
        dmsController.checkDMS();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
