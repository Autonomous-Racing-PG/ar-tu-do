#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import quaternion_from_euler

import math


def reset(position, orientation):
    state = ModelState()
    state.model_name = "racer"
    state.pose.position.x = position[0]
    state.pose.position.y = position[1]
    state.pose.position.z = 0

    q = quaternion_from_euler(orientation, math.pi, 0)
    state.pose.orientation.x = q[0]
    state.pose.orientation.z = q[1]
    state.pose.orientation.w = q[2]
    state.pose.orientation.y = q[3]

    set_model_state(state)


rospy.init_node("reset_car")
rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

reset((0, 0), math.pi)
