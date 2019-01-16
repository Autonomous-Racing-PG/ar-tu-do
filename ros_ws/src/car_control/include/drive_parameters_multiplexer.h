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
    /**
     * @brief Construct a new DriveParametersMultiplexer object and initialize sources for all 
     * publishers of drive parameters
     */
    DriveParametersMultiplexer();
    ~DriveParametersMultiplexer();

    private:
    ros::NodeHandle m_node_handle;

    std::vector<DriveParametersSource*> m_sources;
    DriveParametersSource* m_last_updated_source;
    ros::Publisher m_drive_parameters_publisher;

    /**
     * @brief Determines wheter an updated source will be forwarded to the car controller,
     * based on which source was previously forwarded, whether they are idle or outdated.
     */
    bool validateSource(DriveParametersSource* source);

    /**
     * @brief This function should be called when a source has received a message.
     * It determines if the message should be forwarded and if it should, it sends the message
     * to the car controller.
     */
    void onUpdate(DriveParametersSource* source, const drive_msgs::drive_param::ConstPtr& message);
};
