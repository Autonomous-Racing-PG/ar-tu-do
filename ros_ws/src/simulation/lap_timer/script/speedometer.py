#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates

from collections import namedtuple

Point = namedtuple("Point", ["x", "y"])

last_time = None
last_position = None
last_velocity = 0
max_velocity = 0


def get_distance(a, b):
    return ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5


def model_state_callback(message):
    global last_time, last_position, max_velocity, last_velocity
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

    if last_velocity < 0.01 and velocity < 0.01:
        return
    last_velocity = velocity
    if velocity > max_velocity:
        max_velocity = velocity
    rospy.loginfo(
        "car: {0:.2f} m/s, max: {1:.2f} m/s".format(velocity, max_velocity))


rospy.init_node('speedometer', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)

rospy.spin()
