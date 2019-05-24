#include "drive_parameters_multiplexer.h"

DriveParametersMultiplexer::DriveParametersMultiplexer()
    : m_drive_mode{ DriveMode::LOCKED }
{
    this->m_drive_parameters_publisher = this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1);
    this->m_last_updated_source = NULL;
    auto callback =
        std::bind(&DriveParametersMultiplexer::onUpdate, this, std::placeholders::_1, std::placeholders::_2);

    this->m_sources = {
        std::move(std::make_unique<DriveParametersSource>(&this->m_node_handle, TOPIC_DRIVE_PARAMETERS_KEYBOARD,
                                                          callback, DriveMode::MANUAL, ros::Duration(0.1))),
        std::move(std::make_unique<DriveParametersSource>(&this->m_node_handle, TOPIC_DRIVE_PARAMETERS_JOYSTICK,
                                                          callback, DriveMode::MANUAL, ros::Duration(0.1))),
        std::move(std::make_unique<DriveParametersSource>(&this->m_node_handle, TOPIC_DRIVE_PARAMETERS_AUTONOMOUS,
                                                          callback, DriveMode::AUTONOMOUS, ros::Duration(0.1))),
    };
    this->m_drive_mode_subscriber =
        this->m_node_handle.subscribe<std_msgs::Int32>(TOPIC_DRIVE_MODE, 1,
                                                       &DriveParametersMultiplexer::driveModeCallback, this);
}

// clang-format off
bool DriveParametersMultiplexer::validateSource(DriveParametersSource* source)
{
    ROS_ASSERT_MSG(source != nullptr, "Parameter 'source' must not be null.");

    if (source->getDriveMode() != this->m_drive_mode)
    {
        return false;
    }

    return this->m_last_updated_source == nullptr
        || this->m_last_updated_source == source
        || this->m_last_updated_source->isOutdated()
        || this->m_last_updated_source->getDriveMode() != this->m_drive_mode
        || (!source->isIdle() && this->m_last_updated_source->isIdle());
}
// clang-format on

void DriveParametersMultiplexer::onUpdate(DriveParametersSource* source,
                                          const drive_msgs::drive_param::ConstPtr& message)
{
    ROS_ASSERT_MSG(source != nullptr, "Parameter 'source' must not be null.");
    if (!this->validateSource(source))
    {
        return;
    }
    this->m_drive_parameters_publisher.publish(message);
    this->m_last_updated_source = source;
}

void DriveParametersMultiplexer::driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message)
{
    this->m_drive_mode = (DriveMode)drive_mode_message->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_parameters_multiplexer");
    DriveParametersMultiplexer multiplexer;
    ros::spin();
    return EXIT_SUCCESS;
}
