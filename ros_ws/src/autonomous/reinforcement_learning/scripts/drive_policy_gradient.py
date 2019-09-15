#!/usr/bin/env python

from reinforcement_learning_node import ReinforcementLearningNode
import os
import rospy
from parameters_policy_gradient import Policy, ACTIONS, LASER_SAMPLE_COUNT
import torch
from torch.distributions import Categorical


class PolicyGradientDrivingNode(ReinforcementLearningNode):
    ''' ROS node to drive the car using previously learned
    Policy Gradient weights
    '''

    def __init__(self):
        self.policy = Policy()

        try:
            self.policy.load()
            self.policy.eval()
        except IOError:
            message = "Model parameters for the neural net not found. You need to train it first."
            rospy.logerr(message)
            rospy.signal_shutdown(message)
            exit(1)

        ReinforcementLearningNode.__init__(self, ACTIONS, LASER_SAMPLE_COUNT)

    def on_receive_laser_scan(self, message):
        if self.policy is None:
            return

        state = self.convert_laser_message_to_tensor(message, use_device=False)

        with torch.no_grad():
            action_probabilities = self.policy(state)
        action_distribution = Categorical(action_probabilities)
        action = action_distribution.sample()
        self.perform_action(action)


rospy.init_node('policy_gradient_driving', anonymous=True)
node = PolicyGradientDrivingNode()
rospy.spin()
