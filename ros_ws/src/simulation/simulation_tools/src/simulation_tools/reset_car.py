#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import quaternion_from_euler

import math
import random

from track import track, Point


def set_pose(position, orientation):
    state = ModelState()
    state.model_name = "racer"
    state.pose.position.x = position.x
    state.pose.position.y = position.y
    state.pose.position.z = 0

    q = quaternion_from_euler(orientation, math.pi, 0)
    state.pose.orientation.x = q[0]
    state.pose.orientation.z = q[1]
    state.pose.orientation.w = q[2]
    state.pose.orientation.y = q[3]

    set_model_state(state)


def reset(progress=0, angle=0, offset_from_center=0, forward=True):
    position = track.get_position(progress * track.length, offset_from_center)
    angle += position.angle
    if forward:
        angle += math.pi
    set_pose(position.point, angle)


def reset_random(max_angle=0, max_offset_from_center=0, forward=True):
    reset(random.random(), (random.random() * 2 - 1) * max_angle,
          (random.random() * 2 - 1) * max_offset_from_center, forward)


set_model_state = None


def register_service():
    global set_model_state
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy(
        '/gazebo/set_model_state', SetModelState)


if __name__ == "__main__":
    rospy.init_node("reset_car")
    register_service()
    set_pose(Point(0, 0), math.pi)
