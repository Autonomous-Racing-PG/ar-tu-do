#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
import torch
from parameters import *


class DrivingNode():
    def __init__(self):
        self.scan_indices = None
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)

        self.driver = NeuralCarDriver()
        try:
            self.driver.load()
        except IOError:
            message = "Model parameters for the neural net not found. You need to train it first."
            rospy.logerr(message)
            rospy.signal_shutdown(message)
            exit(1)
    
    def convert_laser_message_to_tensor(self, message):
        if self.scan_indices is None:
            count = (message.angle_max - message.angle_min) / \
                message.angle_increment
            self.scan_indices = [int(i * count / STATE_SIZE) for i in range(STATE_SIZE)]

        values = [message.ranges[i] for i in self.scan_indices]
        values = [v if not math.isinf(v) else 100 for v in values]
        return torch.tensor(values, dtype=torch.float)

    def on_receive_laser_scan(self, message):
        state = self.convert_laser_message_to_tensor(message)
        self.driver.drive(state)


rospy.init_node('evolutionary_driver', anonymous=True)
node = DrivingNode()
rospy.spin()