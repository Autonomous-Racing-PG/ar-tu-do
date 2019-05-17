#!/usr/bin/env python

from qlearning import QLearningNode
import os
import rospy
from parameters import MODEL_FILENAME
import torch


class QLearningDrivingNode(QLearningNode):
    def __init__(self):
        QLearningNode.__init__(self)

        self.policy.load()
        if not os.path.isfile(MODEL_FILENAME):
            message = "Model parameters for the neural net not found. You need to train it first."
            rospy.logerr(message)
            rospy.signal_shutdown(message)
            exit(1)

    def on_receive_laser_scan(self, message):
        state = self.convert_laser_message_to_tensor(message)

        with torch.no_grad():
            action = self.policy(state).max(0)[1].item()
        self.perform_action(action)


rospy.init_node('q_learning_driving', anonymous=True)
node = QLearningDrivingNode()
rospy.spin()
