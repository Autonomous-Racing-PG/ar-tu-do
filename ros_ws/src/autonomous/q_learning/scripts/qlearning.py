#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from collections import deque

import torch

from parameters import *


class QLearningNode():
    def __init__(self):
        self.policy = NeuralQEstimator().to(device)

        self.scan_indices = None
        self.drive_parameters_publisher = rospy.Publisher(
            TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)

        self.last_scan = None
        self.last_scan_time = None

        self.steering_history = deque(maxlen=20)
        self.throttle_history = deque(maxlen=20)
        self.steering_history.append(0)
        self.throttle_history.append(0)

    def perform_action(self, action_index):
        if action_index < 0 or action_index >= len(ACTIONS):
            raise Exception("Invalid action: " + str(action_index))

        angle, velocity = ACTIONS[action_index]
        self.steering_history.append(angle)
        self.throttle_history.append(velocity)
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
        return torch.tensor(values, device=device)

    def get_state(self, laser_message):
        scan = self.convert_laser_message_to_tensor(laser_message)
        scan_time = laser_message.header.stamp.to_sec()

        if self.last_scan is None or abs(scan_time - self.last_scan_time) < 0.0001:
            scan_derivative = torch.zeros(scan.shape, device = device)
        else:
            scan_derivative = (scan - self.last_scan) / (scan_time - self.last_scan_time)

        throttle_average = float(sum(self.throttle_history)) / len(self.throttle_history)
        steering_average = float(sum(self.steering_history)) / len(self.steering_history)
        last_action_average_tensor = torch.tensor([throttle_average, steering_average], device = device)

        self.last_scan = scan
        self.last_scan_time = scan_time

        state = torch.cat([scan_derivative, scan, last_action_average_tensor])
        return state.detach()

    def on_receive_laser_scan(self, message):
        raise Exception("on_receive_laser_scan is not implemented.")


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
