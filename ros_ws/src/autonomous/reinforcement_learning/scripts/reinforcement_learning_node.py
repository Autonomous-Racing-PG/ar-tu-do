#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param

import torch

from topics import TOPIC_DRIVE_PARAMETERS, TOPIC_SCAN


class ReinforcementLearningNode():
    ''' Abstract class for methods that are used both
    during training and during driving.
    '''

    def __init__(self, actions, laser_sample_count):
        self.scan_indices = None
        self.laser_sample_count = laser_sample_count
        self.actions = actions
        self.drive_parameters_publisher = rospy.Publisher(
            TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)

    def perform_action(self, action_index):
        if action_index < 0 or action_index >= len(self.actions):
            raise Exception("Invalid action: " + str(action_index))

        angle, velocity = self.actions[action_index]
        message = drive_param()
        message.angle = angle
        message.velocity = velocity
        self.drive_parameters_publisher.publish(message)

    def convert_laser_message_to_tensor(self, message, use_device=True):
        if self.scan_indices is None:
            self.scan_indices = [int(i * (len(message.ranges) - 1) / (self.laser_sample_count - 1)) for i in range(self.laser_sample_count)]  # nopep8

        values = [message.ranges[i] for i in self.scan_indices]
        values = [v if not math.isinf(v) else 100 for v in values]

        return torch.tensor(
            values,
            device=device if use_device else None,
            dtype=torch.float)

    def on_receive_laser_scan(self, message):
        raise Exception("on_receive_laser_scan is not implemented.")


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
