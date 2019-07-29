
import numpy as np
from math import atan2
from collections import namedtuple

from circle_fit import hyper_fit

Point = namedtuple("Point", ["x", "y"])


class Circle():
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def create_array(self, start_angle, end_angle, sample_count=50):
        points = np.zeros((sample_count, 2))
        angles = np.linspace(start_angle, end_angle, sample_count)
        points[:, 0] = self.center.x + np.sin(angles) * self.radius
        points[:, 1] = self.center.y + np.cos(angles) * self.radius
        return points

    def get_angle(self, point):
        return atan2(point.x - self.center.x, point.y - self.center.y)

    def get_closest_point(self, point):
        x = point.x - self.center.x
        y = point.y - self.center.y
        distance = (x**2 + y**2) ** 0.5
        return Point(
            self.center.x + x * self.radius / distance,
            self.center.y + y * self.radius / distance
        )

    @staticmethod
    def fit(points):
        center_x, center_y, radius, _ = hyper_fit(points)
        return Circle(Point(center_x, center_y), radius)
