#include "chase_cam.h"

ignition::math::Pose3d convertPoseMessageToPose(const gazebo::msgs::Pose pose_message)
{
    auto position = pose_message.position();
    auto orientation = pose_message.orientation();

    return ignition::math::Pose3d(position.x(), position.y(), position.z(), orientation.w(), orientation.x(),
                                  orientation.y(), orientation.z());
}

ChaseCam::ChaseCam()
{
    this->m_gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->m_gazebo_node->Init();

    this->m_camera_pose_publisher = this->m_gazebo_node->Advertise<gazebo::msgs::Pose>(TOPIC_CAMERA_POSE);
    this->m_racer_pose_subscriber =
        this->m_gazebo_node->Subscribe(TOPIC_GAZEBO_POSES, &ChaseCam::gazeboPosesCallback, this);
}

void ChaseCam::gazeboPosesCallback(ConstPosesStampedPtr& message)
{
    for (int i = 0; i < message->pose_size(); i++)
    {
        auto pose_message = message->pose(i);
        if (pose_message.name() == CAR_NAME)
        {
            auto pose = convertPoseMessageToPose(pose_message);
            this->updateCamera(pose);
            return;
        }
    }
}

void ChaseCam::updateCamera(ignition::math::Pose3d& car_pose)
{
    auto car_rotation = ignition::math::Quaterniond(0, 0, car_pose.Rot().Yaw());

    this->m_last_position = car_pose.Pos() * 0.1 + this->m_last_position * 0.9;
    this->m_last_rotation = ignition::math::Quaterniond::Slerp(0.08, this->m_last_rotation, car_rotation, true);

    auto camera_position = this->m_last_position + this->m_last_rotation * CAMERA_OFFSET;

    auto camera_pose = ignition::math::Pose3d(camera_position, this->m_last_rotation);
    this->publishCameraPose(camera_pose);
}

void ChaseCam::publishCameraPose(ignition::math::Pose3d& pose)
{
    gazebo::msgs::Pose message;
    gazebo::msgs::Set(&message, pose);
    this->m_camera_pose_publisher->Publish(message);
}

int main(int argc, char** argv)
{
    gazebo::client::setup(argc, argv);
    ChaseCam chase_cam;

    while (true)
    {
        gazebo::common::Time::MSleep(1000);
    }

    gazebo::client::shutdown();
    return EXIT_SUCCESS;
}
