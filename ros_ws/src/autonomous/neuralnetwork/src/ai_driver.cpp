#include "ai_driver.h"

AiDriver::AiDriver()
{
    ros::NodeHandle private_node_handle("~");

    std::string config_folder;
    std::string config_file;
    private_node_handle.getParam(PARAMETER_CONFIG_FOLDER, config_folder);
    private_node_handle.getParam(PARAMETER_CONFIG_FILE, config_file);

    m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN_SUBSCRIBE, 1, &AiDriver::lidarCallback, this);
    m_net_deploy_subscriber = m_node_handle.subscribe<neuralnetwork::net_param>(TOPIC_NET_DEPLOY_SUBSCRIBE, 1,
                                                                                &AiDriver::netDeployCallback, this);

    m_drive_parameter_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS_PUBLISH, 1);

    m_timer = m_node_handle.createTimer(ros::Duration(0.2), &AiDriver::timerCallback, this);

    std::string file_path = config_folder + "/" + config_file;
    if (m_net.create_from_file(file_path))
    {
        ROS_INFO_STREAM("successfully loaded ai driver from " + file_path);
    }
    else
    {
        ROS_WARN_STREAM("could not load " + file_path);
        ROS_INFO_STREAM("initialising network with random weights");
        m_net.create_standard_array(NUM_LAYERS, NET_ARGS);
    }
}

void AiDriver::timerCallback(const ros::TimerEvent&)
{
    update();
}

void AiDriver::publishDriveParameters(fann_type velocity, fann_type angle)
{
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = angle;
    m_drive_parameter_publisher.publish(drive_parameters);
}

void AiDriver::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    for (int i = 0; i < 5; i++)
    {
        m_input[i + 2] = lidar->ranges[LIDAR_INDICES[i]];
    }
}

void AiDriver::netDeployCallback(const neuralnetwork::net_param::ConstPtr& data)
{
    long size = data->size;
    FANN::connection connections[size];
    m_net.get_connection_array(connections);
    for (int i = 0; i < size; i++)
    {
        connections[i].weight = data->weights[i];
    }
}

void AiDriver::update()
{
    // run network
    m_output = m_net.run(m_input);

    // redirecting speed and angle of the output back as inputs
    m_input[0] = m_output[0];
    m_input[1] = m_output[1];

    // publish outputs
    publishDriveParameters(m_output[0], m_output[1]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_driver");
    AiDriver ai_driver;
    ros::spin();
    return EXIT_SUCCESS;
}
