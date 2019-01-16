#include "drive_parameters_source.h"
#include <math.h>

long int getTime()
{
    struct timeval time_struct;
    gettimeofday(&time_struct, NULL);
    return time_struct.tv_sec * 1000 + time_struct.tv_usec / 1000;
}

DriveParametersSource::DriveParametersSource(ros::NodeHandle* node_handle, const char* topic,
                                             DriveParameterCallbackFunction update_callback, int priority,
                                             double timeout)
{
    this->m_drive_parameters_subscriber =
        node_handle->subscribe<drive_msgs::drive_param>(topic, 1, &DriveParametersSource::driveParametersCallback,
                                                        this);
    this->m_priority = priority;
    this->m_timeout = (long int)(timeout * 1000.0);
    this->m_last_update = 0;
    this->m_idle = true;
    this->m_updateCallback = update_callback;
}

void DriveParametersSource::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& message)
{
    this->m_last_update = getTime();
    this->m_idle = fabs(message->velocity) < 0.01 && fabs(message->angle) < 0.01;
    this->m_updateCallback(this, message);
}

bool DriveParametersSource::isOutdated()
{
    if (this->m_last_update == 0)
    {
        return true;
    }
    return this->m_last_update + this->m_timeout < getTime();
}

bool DriveParametersSource::isIdle()
{
    return this->m_idle;
}

int DriveParametersSource::getPriority()
{
    return this->m_priority;
}