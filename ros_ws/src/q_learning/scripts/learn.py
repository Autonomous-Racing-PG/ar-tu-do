#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates

rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

current_position = None
start_position = None

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

def get_distance_travelled():
    if current_position is None or start_position is None:
        return 0
    return ((current_position.x - start_position.x)**2 + (current_position.y - start_position.y)**2)**0.5

def reset_run(_ = None):
    global start_position

    distance_travelled = get_distance_travelled()
    if distance_travelled < 0.1:
        return

    rospy.loginfo("Distance travelled: " + str(distance_travelled) + "m")
    start_position = None
    respawn()

def model_state_callback(message):
    global current_position, start_position
    if len(message.pose) < 2:
        return
    current_position = message.pose[1].position
    if start_position == None:
        start_position = current_position


rospy.init_node('learn', anonymous=True)
rospy.Subscriber("/crash", Empty, reset_run)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rate = rospy.Rate(0.3)
while not rospy.is_shutdown():
    rate.sleep()
