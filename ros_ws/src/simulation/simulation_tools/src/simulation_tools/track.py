# coding: utf-8

import random
import numpy as np
import math
import rospy
import sys

from tf.transformations import euler_from_quaternion

from collections import namedtuple
Point = namedtuple("Point", ["x", "y"])


class TrackPosition():
    def __init__(self, segment, x, y, point, track):
        self.point = point
        self.segment = segment
        self.distance_to_center = x
        self.segment_distance = y
        self.segment_progress = y / track.segment_length[segment]
        self.total_distance = track.cumulative_distance[segment] + y
        self.total_progress = self.total_distance / track.length
        self.angle = math.atan2(
            track.forward[segment, 1], track.forward[segment, 0])

    def __str__(self):
        return "{0:.2f} m ({1:.0f}%), segment {2:d}, to center: {3:.2f} m, track: {4:.0f}°".format(
            self.total_distance,
            self.total_progress * 100,
            self.segment,
            self.distance_to_center,
            self.angle * 180 / math.pi
        )

    def get_relative_angle(self, orientation):
        quaternion = [
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z]
        euler = euler_from_quaternion(quaternion)
        return (euler[0] + self.angle) % (2 * math.pi) - math.pi

    def faces_forward(self, orientation):
        return abs(self.get_relative_angle(orientation)) < math.pi / 2


class Track():
    def __init__(self, points):
        self.points = points[:-1, :]
        self.size = points.shape[0] - 1
        relative = points[1:, :] - points[:-1, :]
        self.segment_length = np.linalg.norm(relative, axis=1)
        self.length = np.sum(self.segment_length)
        self.forward = relative / self.segment_length[:, np.newaxis]
        self.right = np.array(
            [self.forward[:, 1], -self.forward[:, 0]]).transpose()
        self.cumulative_distance = np.zeros(points.shape[0])
        self.cumulative_distance[1:] = np.cumsum(self.segment_length)

    def localize(self, point):
        ''' Returns a TrackPosition object based on a position in world space. '''
        local = np.array([point.x, point.y]) - self.points
        x = local[:, 0] * self.right[:, 0] + local[:, 1] * self.right[:, 1]
        y = local[:, 0] * self.forward[:, 0] + local[:, 1] * self.forward[:, 1]
        distances = np.abs(x)
        distances[(y < 0) | (y > self.segment_length)] = float("Inf")
        segment = np.argmin(distances)
        return TrackPosition(segment, x[segment], y[segment], point, self)

    def get_position(self, total_distance, distance_to_center=0):
        ''' Returns a TrackPosition object based on the on-track distance from the start
        line and the distance to the center of the track. '''
        segment = 0
        while self.cumulative_distance[segment + 1] < total_distance:
            segment += 1
        segment_distance = total_distance - self.cumulative_distance[segment]
        x = self.points[segment, 0] + self.forward[segment, 0] * \
            segment_distance + self.right[segment, 0] * distance_to_center
        y = self.points[segment, 1] + self.forward[segment, 1] * \
            segment_distance + self.right[segment, 1] * distance_to_center
        return TrackPosition(segment, distance_to_center,  # nopep8
            segment_distance, Point(x, y), self)  # nopep8


world_name = rospy.get_param("world_name")

if world_name not in [
    "racetrack_decorated",
    "racetrack_decorated_2",
        "racetrack_decorated_2_big"]:
    print "ERROR: Racetrack not supported by track.py"
    sys.exit(1)

POINTS = np.array((
    (2.64, -0.36),
    (6.08, -0.33),
    (7.64, -0.15),
    (9.10, 0.41),
    (10.31, 1.39),
    (11.13, 2.72),
    (11.56, 4.23),
    (11.67, 5.79),
    (11.65, 12.07),
    (11.14, 13.52),
    (10.05, 14.65),
    (8.73, 15.48),
    (7.18, 15.66),
    (-0.67, 15.70),
    (-2.06, 15.14),
    (-2.39, 13.68),
    (-1.97, 12.11),
    (-1.48, 11.79),
    (-0.73, 11.67),
    (2.41, 11.68),
    (3.88, 11.34),
    (5.05, 10.33),
    (5.60, 8.88),
    (5.44, 7.37),
    (4.78, 5.96),
    (3.76, 4.83),
    (2.39, 4.06),
    (0.86, 3.76),
    (-0.67, 3.98),
    (-2.08, 4.66),
    (-3.31, 5.63),
    (-9.86, 12.40),
    (-12.18, 14.53),
    (-13.41, 15.26),
    (-14.93, 15.60),
    (-16.46, 15.35),
    (-17.66, 14.38),
    (-18.21, 12.93),
    (-18.30, 11.36),
    (-18.11, 9.81),
    (-17.35, 6.88),
    (-17.28, 5.31),
    (-16.91, 3.80),
    (-16.07, 2.48),
    (-14.89, 1.53),
    (-13.44, 0.92),
    (-11.90, 0.65),
    (-9.02, 0.66),
    (-6.00, -0.17),
    (2.64, -0.36)))

if world_name == "racetrack_decorated_2_big":
    POINTS *= 2.5

track = Track(POINTS)
