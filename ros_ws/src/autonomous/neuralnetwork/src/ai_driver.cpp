#include "ai_driver.h"

#include "ai_vector_math.h"
#include <eigen3/Eigen/Dense>

AiDriver::AiDriver()
{
    // debug
    int size = 10;
    Eigen::VectorXd vec = r_binary_mutation(size, 6);
    ROS_INFO_STREAM("======================================================== test output");
    for(int i = 0; i < 10; i++)
    {
        ROS_INFO_STREAM(std::to_string(vec[i]));
    }
    // ***********
    ROS_INFO_STREAM("========================================================");


    ros::NodeHandle private_node_handle("~");

    std::string config_folder;
    std::string config_file;
    private_node_handle.getParam(PARAMETER_CONFIG_FOLDER, config_folder);
    private_node_handle.getParam(PARAMETER_CONFIG_FILE, config_file);

    m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN_SUBSCRIBE, 1, &AiDriver::lidarCallback, this);
    m_net_deploy_subscriber = m_node_handle.subscribe<neuralnetwork::net_param>(TOPIC_NET_DEPLOY_SUBSCRIBE, 1,
                                                                                &AiDriver::netDeployCallback, this);

    m_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS_PUBLISH, 1);

    m_timer = m_node_handle.createTimer(ros::Duration(0.1), &AiDriver::timerCallback, this);

    std::string file_path = config_folder + "/" + config_file;
    if (m_net.create_from_file(file_path))
    {
        ROS_INFO_STREAM("successfully loaded ai driver from " + file_path);
    }
    else
    {
        ROS_ERROR_STREAM("could not load " + file_path);
        ROS_ERROR_STREAM("maybe the folder or reading permission is missing");
        ROS_INFO_STREAM("initialising network with random weights");
        m_net.create_standard_array(NUM_LAYERS, NET_ARGS);
        m_net.randomize_weights((fann_type)(-1.0), (fann_type)(1.0));
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
    m_drive_parameters_publisher.publish(drive_parameters);
}

void AiDriver::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    // std::cout << lidar->range_min << " < " << lidar->ranges[3] << " < " << lidar->range_max << std::endl;
    // std::cout << "angle_min: " << lidar->angle_min << " | angle_max: " << lidar->angle_max << " | angle_increment: "
    // << lidar->angle_increment  <<  " | range_min: " << lidar->range_min << " | range_max: " << lidar->range_max <<
    // std::endl;
    for (int i = 0; i < 5; i++)
    {
        float value = lidar->ranges[LIDAR_INDICES[i]];
        if (value < lidar->range_min)
        {
            value = 0;
        }
        else if (value > lidar->range_max)
        {
            value = lidar->range_max;
        }
        m_input[i + 2] = value;
        // std::cout << m_input[i] << " ";
    }
    // std::cout << std::endl;
}

void AiDriver::netDeployCallback(const neuralnetwork::net_param::ConstPtr& data)
{
    long size = data->size;
    FANN::connection connections[size];
    for (int i = 0; i < size; i++)
    {
        connections[i] = FANN::connection();
        connections[i].weight = data->weights[i];
    }
    m_net.create_standard_array(NUM_LAYERS, NET_ARGS);
    m_net.set_weight_array(connections, size);
}

void AiDriver::update()
{
    // run network
    m_output = m_net.run(m_input);

    // redirecting speed and angle of the output back as inputs
    m_input[0] = m_output[0];
    m_input[1] = m_output[1];

    // publish outputs
    fann_type speed = 1 - (m_output[0] * 2);
    fann_type angle = 1 - (m_output[1] * 2);

    if (speed < -1 || speed > 1)
    {
        ROS_WARN_STREAM("ai_driver speed should be -1 < x < 1 but it is " << std::to_string(speed));
    }
    if (angle < -1 || angle > 1)
    {
        ROS_WARN_STREAM("ai_driver angle should be -1 < x < 1 but it is " << std::to_string(speed));
    }
    publishDriveParameters(speed, angle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_driver");
    AiDriver ai_driver;
    ros::spin();
    return EXIT_SUCCESS;
}
