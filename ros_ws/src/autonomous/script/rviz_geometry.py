import rospy

import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point as PointMessage
from visualization_msgs.msg import Marker
from circle import Point

RVIZ_TOPIC = "/wallfollowing_visualization"
RVIZ_FRAME = "laser"
RVIZ_NAMESPACE = "wall_following"

marker_publisher = rospy.Publisher(RVIZ_TOPIC, Marker, queue_size=1)


def show_line_in_rviz(id, points, color, line_width=0.02):
    message = Marker()
    message.header.frame_id = RVIZ_FRAME
    message.header.stamp = rospy.Time.now()
    message.ns = RVIZ_NAMESPACE
    message.type = Marker.ADD
    message.pose.orientation.w = 1

    message.id = id
    message.type = Marker.LINE_STRIP
    message.scale.x = line_width
    message.color = color
    if isinstance(points, np.ndarray):
        message.points = [PointMessage(points[i, 1], -points[i, 0], 0) for i in range(points.shape[0])]  # nopep8
    elif isinstance(points, list):
        message.points = [PointMessage(point.y, -point.x, 0) for point in points]  # nopep8
    else:
        raise Exception("points should be a numpy array or list of points, but is " + str(type(points)) + ".")  # nopep8

    marker_publisher.publish(message)


def show_circle_in_rviz(circle, wall, id):
    start_angle = circle.get_angle(Point(wall[0, 0], wall[0, 1]))
    end_angle = circle.get_angle(Point(wall[-1, 0], wall[-1, 1]))
    points = circle.create_array(start_angle, end_angle)
    show_line_in_rviz(id, points, color=ColorRGBA(0, 1, 1, 1))
