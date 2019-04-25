
import numpy as np
from math import sqrt

import math

from collections import namedtuple

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
        return math.atan2(point.x - self.center.x, point.y - self.center.y)

    def get_closest_point(self, point):
        x = point.x - self.center.x
        y = point.y - self.center.y
        distance = (x**2 + y**2) ** 0.5
        return Point(
            self.center.x +
            x *
            self.radius /
            distance,
            self.center.y +
            y *
            self.radius /
            distance)


# Based on
# https://github.com/AlliedToasters/circle-fit/blob/master/circle_fit/circle_fit.py
def sigma(coords, x, y, r):
    """Computes Sigma for circle fit."""
    dx, dy, sum_ = 0., 0., 0.

    for i in range(len(coords)):
        dx = coords[i][1] - x
        dy = coords[i][0] - y
        sum_ += (sqrt(dx * dx + dy * dy) - r)**2
    return sqrt(sum_ / len(coords))


# Based on
# https://github.com/AlliedToasters/circle-fit/blob/master/circle_fit/circle_fit.py
def fit(points, IterMax=99):
    X = points[:, 0]
    Y = points[:, 1]

    n = X.shape[0]

    Xi = X - X.mean()
    Yi = Y - Y.mean()
    Zi = Xi * Xi + Yi * Yi

    # compute moments
    Mxy = (Xi * Yi).sum() / n
    Mxx = (Xi * Xi).sum() / n
    Myy = (Yi * Yi).sum() / n
    Mxz = (Xi * Zi).sum() / n
    Myz = (Yi * Zi).sum() / n
    Mzz = (Zi * Zi).sum() / n

    Mz = Mxx + Myy
    Cov_xy = Mxx * Myy - Mxy * Mxy
    Var_z = Mzz - Mz * Mz

    A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz
    A1 = Var_z * Mz + 4. * Cov_xy * Mz - Mxz * Mxz - Myz * Myz
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * \
        (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy
    A22 = A2 + A2

    y = A0
    x = 0.
    for i in range(IterMax):
        Dy = A1 + x * (A22 + 16. * x * x)
        xnew = x - y / Dy
        if xnew == x or not np.isfinite(xnew):
            break
        ynew = A0 + xnew * (A1 + xnew * (A2 + 4. * xnew * xnew))
        if abs(ynew) >= abs(y):
            break
        x, y = xnew, ynew

    det = x * x - x * Mz + Cov_xy
    Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.
    Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.

    x = Xcenter + X.mean()
    y = Ycenter + Y.mean()
    r = sqrt(abs(Xcenter**2 + Ycenter**2 + Mz))

    return Circle(Point(x, y), r)
