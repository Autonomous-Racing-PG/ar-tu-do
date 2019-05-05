#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates, LinkState, LinkStates

from collections import namedtuple

from math import acos

Point = namedtuple("Point", ["x", "y"])

velocity = 0
max_velocity = 0
last_model_state_msg = None


def model_state_callback(message):
    global last_model_state_msg
    last_model_state_msg = message


def model_state_update(message):
    global max_velocity, velocity
    if len(message.pose) < 2:
        return
    velocity = (message.twist[1].linear.x**2 +
                message.twist[1].linear.y**2)**0.5
    if velocity > max_velocity:
        max_velocity = velocity


LINK_NAMES = [
    'racer::left_wheel_front',
    'racer::left_wheel_back',
    'racer::right_wheel_front',
    'racer::right_wheel_back']
WHEEL_RADIUS = 0.05
odometry_velocity = None
last_link_state_msg = None


def link_state_callback(message):
    global last_link_state_msg
    last_link_state_msg = message


def link_state_update(message):
    global odometry_velocity
    indices = [i for i in range(len(message.name))
               if message.name[i] in LINK_NAMES]
    twists = [message.twist[i].angular for i in indices]

    angle_velocities = [(t.x**2 + t.y**2)**0.5
                        for t in twists]                # imperfect approximation, but good enough?

    angular_velocity = sum(angle_velocities) / len(angle_velocities)
    odometry_velocity = angular_velocity * WHEEL_RADIUS  # missing factor of 2?


def show_info():
    if velocity is None or max_velocity is None or odometry_velocity is None:
        return

    rospy.loginfo(
        "car: {0:.2f} m/s, max: {1:.2f} m/s, wheels: {2:.2f} m/s, slip: ".format(
            velocity,
            max_velocity,
            odometry_velocity) +
        "{0:.2f}".format(
            odometry_velocity -
            velocity).rjust(5))


def calculate_velocity(event):
    global last_link_state_msg, last_model_state_msg
    if last_model_state_msg is None or last_link_state_msg is None:
        return
    model_state_update(last_model_state_msg)
    link_state_update(last_link_state_msg)
    show_info()


rospy.init_node('speedometer', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/gazebo/link_states", LinkStates, link_state_callback)
rospy.Timer(rospy.Duration(0, 50000000), calculate_velocity)

rospy.spin()
