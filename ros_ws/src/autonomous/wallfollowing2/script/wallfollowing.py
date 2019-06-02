#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param

from rviz_geometry import show_circle_in_rviz, show_line_in_rviz

import circle
from circle import Point

import math

import numpy as np

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_LASER_SCAN = "/scan"

MIN_THROTTLE = 0.2
MAX_THROTTLE = 1.0

RADIUS_LOWER = 2
RADIUS_UPPER = 30

STEERING_SLOW_DOWN = 4
STEERING_SLOW_DOWN_DEAD_ZONE = 0.2

HIGH_SPEED_STEERING_LIMIT = 0.5
HIGH_SPEED_STEERING_LIMIT_DEAD_ZONE = 0.2

UPDATE_FREQUENCY = 60

MAX_ACCELERATION = 0.4

CORNER_CUTTING = 1.4
STRAIGHT_SMOOTHING = 1.0

PID_VALUES = (4, 0.2, 0.02)


last_speed = 0


class PIDController():
    def __init__(self, p, i, d, anti_windup=0.2):
        self.p = p
        self.i = i
        self.d = d
        self.anti_windup = anti_windup

        self.integral = 0
        self.previous_error = 0

    def update_and_get_correction(self, error, delta_time):
        self.integral += error * delta_time
        if self.integral > self.anti_windup:
            self.integral = self.anti_windup
        elif self.integral < -self.anti_windup:
            self.integral = -self.anti_windup

        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error
        return self.p * error + self.i * self.integral + self.d * derivative


def map(in_lower, in_upper, out_lower, out_upper, value):
    result = out_lower + (out_upper - out_lower) * \
        (value - in_lower) / (in_upper - in_lower)
    return min(out_upper, max(out_lower, result))


def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def laser_callback(message):
    global laser_scan
    laser_scan = message


def get_scan_as_cartesian():
    if laser_scan is None:
        raise Exception("No scan has been received yet.")

    ranges = np.array(laser_scan.ranges)

    inf_mask = np.isinf(ranges)
    if inf_mask.any():
        ranges = ranges[~inf_mask]

    angles = np.linspace(
        laser_scan.angle_min,
        laser_scan.angle_max,
        ranges.shape[0])

    points = np.zeros((ranges.shape[0], 2))
    points[:, 0] = -np.sin(angles) * ranges
    points[:, 1] = np.cos(angles) * ranges

    return points


def find_left_right_border(points, margin_relative=0.1):
    margin = int(points.shape[0] * margin_relative)

    relative = points[margin + 1:-margin, :] - points[margin:-margin - 1, :]
    distances = np.linalg.norm(relative, axis=1)

    return margin + np.argmax(distances) + 1


def follow_walls(left_circle, right_circle):
    global last_speed

    prediction_distance = CORNER_CUTTING + STRAIGHT_SMOOTHING * last_speed

    predicted_car_position = Point(0, prediction_distance)
    left_point = left_circle.get_closest_point(predicted_car_position)
    right_point = right_circle.get_closest_point(predicted_car_position)

    target_position = Point(
        (left_point.x + right_point.x) / 2,
        (left_point.y + right_point.y) / 2)
    error = (target_position.x - predicted_car_position.x) / prediction_distance
    if math.isnan(error) or math.isinf(error):
        error = 0

    steering_angle = pid.update_and_get_correction(
        error, 1.0 / UPDATE_FREQUENCY)

    radius = min(left_circle.radius, right_circle.radius)
    speed_limit_radius = map(RADIUS_LOWER, RADIUS_UPPER, 0, 1, radius)
    speed_limit_error = max(0, 1 + STEERING_SLOW_DOWN_DEAD_ZONE - abs(error) * STEERING_SLOW_DOWN)  # nopep8
    speed_limit_acceleration = last_speed + MAX_ACCELERATION / UPDATE_FREQUENCY

    relative_speed = min(
        speed_limit_error,
        speed_limit_radius,
        speed_limit_acceleration)
    last_speed = relative_speed
    speed = map(0, 1, MIN_THROTTLE, MAX_THROTTLE, relative_speed)
    steering_angle = steering_angle * map(HIGH_SPEED_STEERING_LIMIT_DEAD_ZONE, 1, 1, HIGH_SPEED_STEERING_LIMIT, relative_speed)
    drive(steering_angle, speed)

    show_line_in_rviz(2, [left_point, right_point],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    show_line_in_rviz(3, [Point(0, 0), predicted_car_position],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    show_line_in_rviz(4, [predicted_car_position,
                          target_position], color=ColorRGBA(1, 0.4, 0, 1))


def handle_scan():
    if laser_scan is None:
        return

    points = get_scan_as_cartesian()
    split = find_left_right_border(points)

    right_wall = points[:split:4, :]
    left_wall = points[split::4, :]

    left_circle = circle.fit(left_wall)
    right_circle = circle.fit(right_wall)

    follow_walls(left_circle, right_circle)

    show_circle_in_rviz(left_circle, left_wall, 0)
    show_circle_in_rviz(right_circle, right_wall, 1)


laser_scan = None

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
drive_parameters_publisher = rospy.Publisher(
    TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

rospy.init_node('wallfollowing', anonymous=True)

timer = rospy.Rate(UPDATE_FREQUENCY)

pid = PIDController(*PID_VALUES)

while not rospy.is_shutdown():
    handle_scan()
    timer.sleep()
