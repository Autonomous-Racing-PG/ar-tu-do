#!/usr/bin/env python
# coding: utf-8

from training_node import TrainingNode, device
import random
import math
from parameters_policy_gradient import *
import numpy as np

from torch.autograd import Variable
from torch.distributions import Categorical

import torch

import simulation_tools.reset_car as reset_car
from simulation_tools.track import track

'''
ROS node to train the Policy Gradient model
'''
class PolicyGradientTrainingNode(TrainingNode):
    def __init__(self):
        TrainingNode.__init__(self, Policy(), ACTIONS, LASER_SAMPLE_COUNT, MAX_EPISODE_LENGTH, LEARNING_RATE)

        if CONTINUE:
            self.policy.load()

    def update_policy(self):
        R = 0
        rewards = []

        # Discount future rewards back to the present using gamma
        for r in self.policy.reward_episode[::-1]:
            R = r + DISCOUNT_FACTOR * R
            rewards.insert(0, R)

        # Scale rewards
        rewards = torch.FloatTensor(rewards)
        if rewards.numel() > 1:
            rewards = (rewards - rewards.mean()) / \
                (rewards.std() + np.finfo(np.float32).eps)

        # Calculate loss
        loss = (torch.sum(torch.mul(self.policy.policy_history,
                                    Variable(rewards)).mul(-1), -1))

        # Update network weights
        if loss != 0:
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            # Save episode history counters
            self.policy.loss_history.append(loss.item())
            self.policy.reward_history.append(
                np.sum(self.policy.reward_episode))

        # Reset policy history and rewards
        self.policy.policy_history = Variable(torch.Tensor())
        self.policy.reward_episode = []

    def select_action(self, state):
        # Select an action (0 or 1) by running policy model and choosing based
        # on the probabilities in state
        state = self.state.type(torch.FloatTensor)
        state = self.policy(Variable(state))
        c = Categorical(state)
        action = c.sample()

        # Add log probability of our chosen action to our history
        if self.policy.policy_history.dim() != 0:
            self.policy.policy_history = torch.cat(
                [self.policy.policy_history, c.log_prob(action).view(1)])
        else:
            self.policy.policy_history = (c.log_prob(action))
        return action

    def get_reward(self):
        # The faster the car, the more reward it gets
        scaleForSpeed = 0
        if self.action < 2:
            scaleForSpeed = 0.4
        elif self.action < 5:
            scaleForSpeed = 0.7
        else:
            scaleForSpeed = 1

        track_position = track.localize(self.car_position)
        distance = abs(track_position.distance_to_center)

        # The closer the car is to the middle of the race track, the more
        # reward it gets
        if distance < 0.2:
            return 1 * scaleForSpeed
        elif distance < 0.4:
            return 0.7 * scaleForSpeed
        else:
            return -0.4 * scaleForSpeed

    def on_complete_episode(self):
        self.update_policy()
        TrainingNode.on_complete_episode(self)

    def on_complete_step(self, state, action, reward, next_state):
        self.policy.reward_episode.append(reward)


rospy.init_node('policy_gradient_training', anonymous=True)
node = PolicyGradientTrainingNode()
rospy.spin()
