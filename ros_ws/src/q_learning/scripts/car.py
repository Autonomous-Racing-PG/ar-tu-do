import rospy
from std_msgs.msg import String, Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

import math

import torch

import numpy as np

MAX_DISTANCE = 10


def reset(position, orientation):
    global start_position
    start_position = position

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


def _crash_callback(_):
    for function in _crash_callback_functions:
        function()


def register_crash_callback(function):
    _crash_callback_functions.append(function)


def _model_state_callback(message):
    global current_position, current_orientation
    if len(message.pose) < 2:
        return
    current_position = (message.pose[1].position.x, message.pose[1].position.y)
    current_orientation = (message.pose[1].orientation.x, message.pose[1].orientation.y ,message.pose[1].orientation.z, message.pose[1].orientation.w)


def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def _laser_callback(message):
    global laser_scan
    laser_scan = message


_scan_indices = None


def get_scan(sample_count, device):
    if laser_scan is None:
        return torch.tensor([0 for _ in range(sample_count)],
                            device=device, dtype=torch.float32)

    global _scan_indices
    if _scan_indices is None or len(_scan_indices) != sample_count:
        count = (laser_scan.angle_max - laser_scan.angle_min) / \
            laser_scan.angle_increment
        _scan_indices = (np.arange(sample_count) * count / sample_count).astype(int)

    scan = np.array(laser_scan.ranges, dtype=np.float32)
    values = scan[_scan_indices]
    values[values > MAX_DISTANCE] = MAX_DISTANCE
    return torch.tensor(values, device=device)

def get_car_pos_x():
    return euler_from_quaternion(current_orientation)


current_position = None
current_orientation = None
start_position = None
laser_scan = None

_crash_callback_functions = []

rospy.loginfo("Waiting for gazebo...")
rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

rospy.Subscriber("/crash", Empty, _crash_callback)
rospy.Subscriber("/gazebo/model_states", ModelStates, _model_state_callback)
rospy.Subscriber("/scan", LaserScan, _laser_callback)
drive_parameters_publisher = rospy.Publisher(
    "/commands/drive_param", drive_param, queue_size=1)
