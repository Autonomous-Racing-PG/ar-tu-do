#include "dms_controller.h"

DMSController::DMSController()
{
    this->m_heartbeat_manual_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int64>(TOPIC_HEARTBEAT_MANUAL, 1, &DMSController::heartbeatManualCallback, this);
    this->m_heartbeat_autonomous_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int64>(TOPIC_HEARTBEAT_AUTONOMOUS, 1, &DMSController::heartbeatAutonomousCallback, this);
    this->m_drive_mode_publisher = this->m_node_handle.advertise<std_msgs::Int32>(TOPIC_DRIVE_MODE, 1);
    this->configureParameters();
    this->m_last_heartbeat_manual = std::chrono::steady_clock::time_point::min();
    this->m_last_heartbeat_autonomous = std::chrono::steady_clock::time_point::min();
}

void DMSController::spin()
{
    ros::Rate loop(this->m_update_frequency);
    while (ros::ok())
    {
        this->publishDriveMode();
        ros::spinOnce();
        loop.sleep();
    }
}

DriveMode DMSController::getDriveMode() {
    auto current_time = std::chrono::steady_clock::now();

    if (this->m_last_heartbeat_manual + this->m_expiration_time > current_time) {
        return DriveMode::MANUAL;
    }
    if (this->m_last_heartbeat_autonomous + this->m_expiration_time > current_time) {
        return DriveMode::AUTONOMOUS;
    }
    return DriveMode::LOCKED;
}

void DMSController::publishDriveMode()
{    
    std_msgs::Int32 drive_mode_message;
    drive_mode_message.data = (int)this->getDriveMode();
    this->m_drive_mode_publisher.publish(drive_mode_message);
}

void DMSController::heartbeatManualCallback(const std_msgs::Int64::ConstPtr& message)
{
    std::chrono::milliseconds time_since_epoch(message->data);
    this->m_last_heartbeat_manual = std::chrono::time_point<std::chrono::steady_clock>(time_since_epoch);
}

void DMSController::heartbeatAutonomousCallback(const std_msgs::Int64::ConstPtr& message)
{
    std::chrono::milliseconds time_since_epoch(message->data);
    this->m_last_heartbeat_autonomous = std::chrono::time_point<std::chrono::steady_clock>(time_since_epoch);
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
