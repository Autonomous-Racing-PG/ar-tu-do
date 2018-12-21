#include <racer_odometry.h>

RacerOdometry::RacerOdometry()
{
    m_gazebo_subscriber =
        m_node_handle.subscribe<gazebo_msgs::LinkStates>(TOPIC_GAZEBO_ODOM, 1, &RacerOdometry::robotPoseUpdate, this);

    m_odom_publisher = m_node_handle.advertise<nav_msgs::Odometry>(TOPIC_ODOM, 1);

    m_last_received_pose = geometry_msgs::Pose();
    m_last_received_twist = geometry_msgs::Twist();
    m_last_received_stamp = ros::Time();

    m_timer = m_node_handle.createTimer(ros::Duration(0.05), boost::bind(&RacerOdometry::timerCallback, this, _1));
}

void RacerOdometry::robotPoseUpdate(const gazebo_msgs::LinkStates::ConstPtr& links)
{
    for (uint i = 0; i < links->name.size(); i++)
    {
        if (links->name.at(i) == BASE_LINK_NAME)
        {
            m_last_received_pose = links->pose.at(i);
            m_last_received_twist = links->twist.at(i);
            m_last_received_stamp = ros::Time::now();
            return;
        }
    }
}

void RacerOdometry::timerCallback(const ros::TimerEvent&)
{
    nav_msgs::Odometry msg;
    msg.header.stamp = m_last_received_stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.pose.pose = m_last_received_pose;
    msg.twist.twist = m_last_received_twist;
    m_odom_publisher.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "racer_odometry");
    RacerOdometry racerOdometry;
    ros::spin();
    return EXIT_SUCCESS;
}
