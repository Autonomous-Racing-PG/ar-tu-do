#pragma once

#include "rviz_geometry_publisher.h"
#include <cmath>
#include <cstdlib>

class Wall
{
    public:
    float m_angle1;
    float m_angle2;
    float m_range1;
    float m_range2;

    Wall(float angle1, float angle2, float range1, float range2);

    float getAngle();

    float predictDistance(float distancce_to_current_position);

    void draw(RvizGeometryPublisher& geometry, int id, std_msgs::ColorRGBA color);
};