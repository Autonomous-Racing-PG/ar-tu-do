#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


def respawn():
    state = ModelState()
    state.model_name = "racer"
    state.pose.position.x = 0
    state.pose.position.y = 0
    state.pose.position.z = 0
    state.twist.linear.x = 0
    state.twist.linear.y = 0
    state.twist.linear.z = 0

    set_model_state(state)

def crash():
    rospy.loginfo("Crashed.")
    respawn()


rospy.init_node('learn', anonymous=True)
rospy.Subscriber("/crash", Empty, crash)
rate = rospy.Rate(0.3)
while not rospy.is_shutdown():
    rate.sleep()
