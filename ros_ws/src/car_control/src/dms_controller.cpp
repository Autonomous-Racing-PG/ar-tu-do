#include "dms_controller.h"

#include "car_control.h"

DMSController::DMSController()
{
    this->m_heartbeat_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int64>(TOPIC_DMS_HEARTBEAT, 1, &DMSController::heartbeatCallback, this);
    this->m_unlock_motor_publisher = this->m_node_handle.advertise<std_msgs::Bool>(TOPIC_UNLOCK_MOTOR, 1);
    this->configureParameters();
    this->m_last_heartbeat_received = std::chrono::steady_clock::time_point::min();
}

void DMSController::spin()
{
    ros::Rate loop(this->m_update_frequency);
    while (ros::ok())
    {
        this->publishUnlockMotor();
        ros::spinOnce();
        loop.sleep();
    }
}

void DMSController::publishUnlockMotor()
{
    auto current_time = std::chrono::steady_clock::now();

    std_msgs::Bool unlock_motor_message;
    unlock_motor_message.data = this->m_last_heartbeat_received + this->m_expiration_time > current_time;
    this->m_unlock_motor_publisher.publish(unlock_motor_message);
}

void DMSController::heartbeatCallback(const std_msgs::Int64::ConstPtr& dms_message)
{
    std::chrono::milliseconds time_since_epoch(dms_message->data);
    this->m_last_heartbeat_received = std::chrono::time_point<std::chrono::steady_clock>(time_since_epoch);
}

void DMSController::configureParameters()
{
    ros::NodeHandle private_node_handle("~");

    private_node_handle.getParam(PARAMETER_DMS_CHECK_RATE, this->m_update_frequency);
    if (this->m_update_frequency <= 0 || this->m_update_frequency > 1000)
    {
        ROS_WARN_STREAM("dms_check_rate should be between 0 and 1000. Your value: " << this->m_update_frequency
                                                                                    << ", using default: 20.");
        this->m_update_frequency = 20;
    }
    int expiration_ms;
    private_node_handle.getParam(PARAMETER_DMS_EXPIRATION, expiration_ms);
    if (expiration_ms <= 0 || expiration_ms > 1000)
    {
        ROS_WARN_STREAM("dms_expiration should be between 0 and 1000. Your value: " << expiration_ms
                                                                                    << ", using default: 100.");
        expiration_ms = 100;
    }
    this->m_expiration_time = std::chrono::duration<double>(expiration_ms / 1000.0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dms_controller");
    DMSController dmsController;
    dmsController.spin();

    return EXIT_SUCCESS;
}
