#pragma once

#include <ros/ros.h>

#include "car_control.h"
#include "drive_parameters_source.h"
#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <vector>

/*
*  This node subscribes to all publishers drive_param messages and selects one to forward to the car controller
*/
class DriveParametersMultiplexer
{
    public:
    DriveParametersMultiplexer();
    ~DriveParametersMultiplexer();

    private:
    ros::NodeHandle m_node_handle;

    std::vector<DriveParametersSource*> m_sources;
    DriveParametersSource* m_last_updated_source;
    ros::Publisher m_drive_parameters_publisher;

    bool validateSource(DriveParametersSource* source);
    void onUpdate(DriveParametersSource* source, const drive_msgs::drive_param::ConstPtr& message);
};
