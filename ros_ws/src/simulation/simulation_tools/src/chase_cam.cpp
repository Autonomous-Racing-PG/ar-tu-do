#include "chase_cam.h"

ChaseCam::ChaseCam()
{
    this->m_gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->m_gazebo_node->Init();

    this->m_camera_pose_publisher = this->m_gazebo_node->Advertise<gazebo::msgs::Pose>(TOPIC_CAMERA_POSE);

    while (true) {
        ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);
        gazebo::msgs::Pose message;
        gazebo::msgs::Set(&message, pose);
        this->m_camera_pose_publisher->Publish(message, true);
        std::cout << "Message was published." << std::endl;
        ros::Duration(2).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chase_cam");
    gazebo::client::setup(argc, argv);

    ChaseCam chase_cam;

    ros::spin();

    gazebo::client::shutdown();
    return EXIT_SUCCESS;
}
