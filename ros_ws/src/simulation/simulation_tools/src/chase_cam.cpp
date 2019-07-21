#include "chase_cam.h"


ignition::math::Pose3d convertPoseMessageToPose(const gazebo::msgs::Pose pose_message) {
    auto position = pose_message.position();
    auto orientation = pose_message.orientation();

    return ignition::math::Pose3d(
        position.x(),
        position.y(),
        position.z(),
        orientation.w(),
        orientation.x(),
        orientation.y(),
        orientation.z()
    );
}

ChaseCam::ChaseCam()
{
    this->m_gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->m_gazebo_node->Init();

    this->m_camera_pose_publisher = this->m_gazebo_node->Advertise<gazebo::msgs::Pose>(TOPIC_CAMERA_POSE);
    this->m_racer_pose_subscriber =
        this->m_gazebo_node->Subscribe(TOPIC_GAZEBO_POSES, &ChaseCam::gazeboPosesCallback, this);
}

void ChaseCam::gazeboPosesCallback(ConstPosesStampedPtr& message) {
    const gazebo::msgs::Pose* car_pose_message = nullptr;
    for (int i = 0; i < message->pose_size(); i++) {
        const gazebo::msgs::Pose& pose = message->pose(i);
        if (pose.name() == CAR_NAME) {
            car_pose_message = &pose;
            break;
        }
    }

    if (car_pose_message == nullptr) {
        return;
    }
    ignition::math::Pose3d car_pose = convertPoseMessageToPose(*car_pose_message);

    ignition::math::Vector3d camera_position = car_pose.Pos() + car_pose.Rot() * CAMERA_OFFSET;
    ignition::math::Quaterniond camera_orientation = car_pose.Rot();
    ignition::math::Pose3d camera_pose(camera_position, camera_orientation);    
    this->publishCameraPose(camera_pose);
}

void ChaseCam::publishCameraPose(ignition::math::Pose3d& pose) {
    gazebo::msgs::Pose message;
    gazebo::msgs::Set(&message, pose);
    this->m_camera_pose_publisher->Publish(message);
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
