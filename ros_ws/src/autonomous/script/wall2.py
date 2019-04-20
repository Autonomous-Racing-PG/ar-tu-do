#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
import circle_fit as cf

import numpy as np

import math

from collections import namedtuple

Point = namedtuple("Point", ["x", "y"])

class Circle():
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def create_array(self, sample_count=200):
        points = np.zeros((sample_count, 2))
        points[:, 0] = self.center.x - np.sin(np.arange(0, math.pi * 2, math.pi * 2 / sample_count)) * self.radius
        points[:, 1] = self.center.y + np.cos(np.arange(0, math.pi * 2, math.pi * 2 / sample_count)) * self.radius
        return points

    def get_closest_point(self, point):
        x = point.x - self.center.x
        y = point.y - self.center.y
        distance = (x**2 + y**2) ** 0.5
        return Point(self.center.x + x * self.radius / distance, self.center.y + y * self.radius / distance)

class PIDController():
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

        self.integral = 0
        self.previous_error = 0
    
    def update_and_get_correction(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error
        return self.p * error + self.i * self.integral + self.d * derivative

def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def laser_callback(message):
    global laser_scan
    laser_scan = message


def fit_circle(points):
    center_x, center_y, _, _ = cf.hyper_fit(points)
    center = Point(center_x, center_y)
    radius = np.average(np.linalg.norm(points - [center.x, center.y], axis=1))
    return Circle(center, radius)

def get_scan_as_cartesian():
    if laser_scan is None:
        raise Exception("No scan has been received yet.")
    
    ranges = np.array(laser_scan.ranges)
    angles = np.arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)

    points = np.zeros((ranges.shape[0], 2))
    points[:, 0] = -np.sin(angles) * ranges
    points[:, 1] = np.cos(angles) * ranges

    return points

def find_left_right_border(points, margin_relative = 0.1):
    margin_relative = 0.1
    margin = int(points.shape[0] * margin_relative)
    
    relative = points[margin + 1:-margin, :] - points[margin:-margin-1,:]
    distances = np.linalg.norm(relative, axis=1)

    return margin + np.argmax(distances) + 1


def follow_walls(left_circle, right_circle):
    predicted_car_position = Point(0, 1.5)
    left_point = left_circle.get_closest_point(predicted_car_position)
    right_point = right_circle.get_closest_point(predicted_car_position)
    
    target_position = Point((left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2)
    error = target_position.x - predicted_car_position.x
    if math.isnan(error) or math.isinf(error):
        error = 0

    steering_angle = pid.update_and_get_correction(error, 1.0 / UPDATE_FREQUENCY)

    SLOW = 0.15
    FAST = 1.0

    relative_speed = 0.1

    radius = min(left_circle.radius, right_circle.radius)
    
    RADIUS_SMALL = 2
    RADIUS_BIG = 10

    absolute_error = abs(error)
    relative_speed = max(0.0, min(1.0, 1.2 - absolute_error * 1.7, (radius - RADIUS_SMALL) / (RADIUS_BIG - RADIUS_SMALL)))

    drive(steering_angle * (1.0 - relative_speed), SLOW + (FAST - SLOW) * relative_speed)


def handle_scan():
    if laser_scan is None:
        return
    
    points = get_scan_as_cartesian()
    split = find_left_right_border(points)        

    right_wall = points[:split, :]
    left_wall = points[split:, :]

    left_circle = fit_circle(left_wall)
    right_circle = fit_circle(right_wall)
    
    follow_walls(left_circle, right_circle)
    

laser_scan = None

rospy.Subscriber("/scan", LaserScan, laser_callback)
drive_parameters_publisher = rospy.Publisher(
    "/input/drive_param/wallfollowing", drive_param, queue_size=1)


rospy.init_node('wall2', anonymous=True)

UPDATE_FREQUENCY = 60

timer = rospy.Rate(UPDATE_FREQUENCY)

pid = PIDController(1.5, 0.01, 0.01)

while not rospy.is_shutdown():
    handle_scan()
    timer.sleep()