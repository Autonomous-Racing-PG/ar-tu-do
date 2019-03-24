#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from random import randint

ACTIONS = [(-0.7, 0.2), (-0.3, 0.3), (0, 0.4), (0.3, 0.3), (0.7, 0.2), (0.0, 0.6)]
LASER_SAMPLE_COUNT = 16
MAX_RANGE = 10

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

def publish_drive_parameters(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)

def drive(action_index):
    if action_index < 0 or action_index >= len(ACTIONS):
        raise Exception("Invalid action: " + str(action_index))
    publish_drive_parameters(*ACTIONS[action_index])

def get_random_action():
    return randint(0, len(ACTIONS) - 1)

def laser_callback(message):
    global laser_scan
    laser_scan = message

def get_state():
    if laser_scan == None:
        return [0 for _ in range(LASER_SAMPLE_COUNT)]
    count = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment
    indices = [int(i * count / LASER_SAMPLE_COUNT) for i in range(LASER_SAMPLE_COUNT)]
    return [min(1, laser_scan.ranges[i] / MAX_RANGE) ** 0.5 for i in indices]

current_position = None
start_position = None
laser_scan = None

rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

rospy.init_node('learn', anonymous=True)
rospy.Subscriber("/crash", Empty, reset_run)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/scan", LaserScan, laser_callback)
drive_parameters_publisher = rospy.Publisher("/commands/drive_param", drive_param, queue_size=1)
respawn()
rate = rospy.Rate(5)
while not rospy.is_shutdown():
    drive(get_random_action())
    rate.sleep()
