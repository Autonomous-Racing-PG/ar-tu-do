#include "drive_parameters_multiplexer.h"

DriveParametersMultiplexer::DriveParametersMultiplexer()
{
    this->m_drive_parameters_publisher = this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 1);
    this->m_last_updated_source = NULL;
    auto callback = std::bind(&DriveParametersMultiplexer::onUpdate, this, std::placeholders::_1, std::placeholders::_2);
    this->m_sources.push_back(new DriveParametersSource(&this->m_node_handle, "/set/drive_param_keyboard", callback, 1, 0.1));
    this->m_sources.push_back(new DriveParametersSource(&this->m_node_handle, "/set/drive_param_joystick", callback, 1, 0.1));
    this->m_sources.push_back(new DriveParametersSource(&this->m_node_handle, "/set/drive_param_wallfollowing", callback, 0, 0.1));
}

DriveParametersMultiplexer::~DriveParametersMultiplexer()
{
    for (auto source : this->m_sources) {
        delete source;
    }
}


bool DriveParametersMultiplexer::validateSource(DriveParametersSource* source) {
    return this->m_last_updated_source == NULL
        || this->m_last_updated_source == source
        || this->m_last_updated_source->isOutdated()
        || (!source->isIdle() && this->m_last_updated_source->isIdle())
        || (!source->isIdle() && this->m_last_updated_source->getPriority() < source->getPriority());
}

void DriveParametersMultiplexer::onUpdate(DriveParametersSource* source, const drive_msgs::drive_param::ConstPtr& message) {
    if (!this->validateSource(source)) {
        return;
    }
    this->m_drive_parameters_publisher.publish(message);
    this->m_last_updated_source = source;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_parameters_multiplexer");
    DriveParametersMultiplexer multiplexer;
    ros::spin();
    return EXIT_SUCCESS;
}
