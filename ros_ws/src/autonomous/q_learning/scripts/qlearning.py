#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param

import torch

from parameters import *


class QLearningNode():
    def __init__(self):
        self.policy = NeuralQEstimator().to(device)

        self.scan_indices = None
        self.drive_parameters_publisher = rospy.Publisher(
            TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)

    def perform_action(self, action_index):
        if action_index < 0 or action_index >= len(ACTIONS):
            raise Exception("Invalid action: " + str(action_index))

        angle, velocity = ACTIONS[action_index]
        message = drive_param()
        message.angle = angle
        message.velocity = velocity
        self.drive_parameters_publisher.publish(message)

    def convert_laser_message_to_tensor(self, message):
        if self.scan_indices is None:
            global scan_indices
            count = (message.angle_max - message.angle_min) / \
                message.angle_increment
            scan_indices = [int(i * count / LASER_SAMPLE_COUNT)
                            for i in range(LASER_SAMPLE_COUNT)]

        values = [message.ranges[i] for i in scan_indices]
        values = [v if not math.isinf(v) else 100 for v in values]
        return torch.tensor(values, device=device, dtype=torch.float)

    def on_receive_laser_scan(self, message):
        raise Exception("on_receive_laser_scan is not implemented.")


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
