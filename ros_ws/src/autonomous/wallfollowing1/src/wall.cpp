#include "wall.h"

Wall::Wall(float angle1, float angle2, float range1, float range2)
    : m_angle1{ angle1 }
    , m_angle2{ angle2 }
    , m_range1{ range1 }
    , m_range2{ range2 }
{
}

float Wall::getAngle()
{
    return std::atan((m_range1 * std::cos(std::abs(m_angle1 - m_angle2)) - m_range2) / m_range1 *
                     std::sin(std::abs(m_angle1 - m_angle2)));
}

float Wall::predictDistance(float distancce_to_current_position)
{
    float angle = this->getAngle();
    float currentWallDistance = m_range2 * std::cos(angle);
    return currentWallDistance + distancce_to_current_position * std::sin(angle);
}

void Wall::draw(RvizGeometryPublisher& geometry, int id, std_msgs::ColorRGBA color)
{
    geometry.drawLine(id, createPoint(m_range1 * std::cos(m_angle1), m_range1 * std::sin(m_angle1), 0.0f),
                      createPoint(m_range2 * std::cos(m_angle2), m_range2 * std::sin(m_angle2), 0.0f), color, 0.03);
}
