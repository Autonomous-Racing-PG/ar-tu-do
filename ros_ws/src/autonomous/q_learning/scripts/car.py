import rospy
from std_msgs.msg import String, Empty
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from tf.transformations import quaternion_from_euler

import math

import torch

from simulation_tools.track import track
from simulation_tools.reset_car import Point

def _crash_callback(_):
    for function in _crash_callback_functions:
        function()


def register_crash_callback(function):
    _crash_callback_functions.append(function)


def _model_state_callback(message):
    global current_position
    if len(message.pose) < 2:
        return
    current_position = Point(message.pose[1].position.x, message.pose[1].position.y)


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
        _scan_indices = [int(i * count / sample_count)
                         for i in range(sample_count)]

    values = [laser_scan.ranges[i] for i in _scan_indices]
    values = [v if not math.isinf(v) and not math.isnan(v) else 100 for v in values]
    return torch.tensor(values, device=device)

def get_distance_to_track_center():
    track_position = track.localize(current_position)
    return track_position.distance_to_center


current_position = None
laser_scan = None

_crash_callback_functions = []

rospy.Subscriber("/crash", Empty, _crash_callback)
rospy.Subscriber("/gazebo/model_states", ModelStates, _model_state_callback)
rospy.Subscriber("/scan", LaserScan, _laser_callback)
drive_parameters_publisher = rospy.Publisher(
    "/commands/drive_param", drive_param, queue_size=1)
