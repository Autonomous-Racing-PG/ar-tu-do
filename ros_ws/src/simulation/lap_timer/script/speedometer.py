#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates, LinkState, LinkStates

from collections import namedtuple

from math import acos

Point = namedtuple("Point", ["x", "y"])

last_time = None
last_position = None
velocity = 0
max_velocity = 0


def get_distance(a, b):
    return ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5


def model_state_callback(message):
    global last_time, last_position, max_velocity, velocity
    if len(message.pose) < 2:
        return

    position = Point(message.pose[1].position.x, message.pose[1].position.y)
    now = rospy.Time.now()

    if last_time is None:
        last_time = now
        last_position = position
        return

    duration = abs((last_time - now).to_sec())
    if duration < 0.03:
        return
    last_time = now

    distance = get_distance(position, last_position)
    last_position = position
    velocity = distance / duration

    if velocity > max_velocity:
        max_velocity = velocity

    show_info()


LINK_NAMES = [
    'racer::left_wheel_front',
    'racer::left_wheel_back',
    'racer::right_wheel_front',
    'racer::right_wheel_back']
WHEEL_RADIUS = 0.05
last_orientations = None
last_time_angular = None
odometry_velocity = None


def get_angle(q1, q2):
    return acos((q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z))


def link_state_callback(message):
    global last_orientations, last_time_angular, odometry_velocity
    indices = [i for i in range(len(message.name))
               if message.name[i] in LINK_NAMES]
    orientations = [message.pose[i].orientation for i in indices]
    if last_orientations is None:
        last_orientations = orientations
        return

    now = rospy.Time.now()

    if last_time_angular is None:
        last_time_angular = now
        return

    duration = abs((last_time_angular - now).to_sec())
    if duration < 0.03:
        return

    last_time_angular = now

    angles = [get_angle(last_orientations[i], orientations[i])
              for i in range(len(orientations))]
    last_orientations = orientations
    angle = sum(angles) / len(angles)
    angular_velocity = angle / duration
    odometry_velocity = angular_velocity * WHEEL_RADIUS * 2


idle = True


def show_info():
    global idle
    if velocity is None or max_velocity is None or odometry_velocity is None:
        return
    if abs(odometry_velocity) < 0.01:
        if idle:
            return
        else:
            idle = True
    else:
        idle = False

    rospy.loginfo(
        "car: {0:.2f} m/s, max: {1:.2f} m/s, wheels: {2:.2f} m/s, slip: ".format(
            velocity,
            max_velocity,
            odometry_velocity) +
        "{0:.2f}".format(
            odometry_velocity -
            velocity).rjust(5))


rospy.init_node('speedometer', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/gazebo/link_states", LinkStates, link_state_callback)

rospy.spin()
