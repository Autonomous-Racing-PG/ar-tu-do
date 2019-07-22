#include "rviz_geometry_publisher.h"

RvizGeometryPublisher::RvizGeometryPublisher(ros::NodeHandle node_handle, const std::string& topic,
                                             const std::string& frame)
    : m_frame(frame)
{
    this->m_marker_publisher = node_handle.advertise<visualization_msgs::Marker>(topic, 10);
}

void RvizGeometryPublisher::drawLine(int id, geometry_msgs::Point point1, geometry_msgs::Point point2,
                                     std_msgs::ColorRGBA color, float width)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = this->m_frame;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = width;
    line_list.color = color;

    line_list.points.push_back(point1);
    line_list.points.push_back(point2);

    this->m_marker_publisher.publish(line_list);
}

geometry_msgs::Point createPoint(float x, float y, float z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::ColorRGBA createColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}