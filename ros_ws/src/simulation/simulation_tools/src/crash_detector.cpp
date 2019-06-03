#include "crash_detector.h"

CrashDetector::CrashDetector()
{
    this->m_crash_publisher = this->m_ros_node_handle.advertise<std_msgs::Empty>(TOPIC_CRASH, 1);
    this->m_gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->m_gazebo_node->Init();
    this->m_walls_sensor_subscriber =
        this->m_gazebo_node->Subscribe(TOPIC_GAZEBO_SENSOR_WALLS, &CrashDetector::gazeboTopicCallback, this);
    this->m_decoration_sensor_subscriber =
        this->m_gazebo_node->Subscribe(TOPIC_GAZEBO_SENSOR_DECORATION, &CrashDetector::gazeboTopicCallback, this);
}

void CrashDetector::gazeboTopicCallback(ConstContactsPtr& gazebo_message)
{
    if (gazebo_message->contact_size() != 0)
    {
        std_msgs::Empty ros_message;
        this->m_crash_publisher.publish(ros_message);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crash_detector");
    gazebo::client::setup(argc, argv);

    CrashDetector crash_detector;

    ros::spin();

    gazebo::client::shutdown();
    return EXIT_SUCCESS;
}
