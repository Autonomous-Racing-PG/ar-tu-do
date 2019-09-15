#!/usr/bin/env python
# coding: utf-8

from training_node import TrainingNode
import random
import math
from parameters_policy_gradient import *
import numpy as np

from torch.distributions import Categorical

import torch

import simulation_tools.reset_car as reset_car
from simulation_tools.track import track

FLOAT_EPS = np.finfo(np.float32).eps


class PolicyGradientTrainingNode(TrainingNode):
    ''' ROS node to train the Policy Gradient model
    '''

    def __init__(self):
        TrainingNode.__init__(
            self,
            Policy(),
            ACTIONS,
            LASER_SAMPLE_COUNT,
            MAX_EPISODE_LENGTH,
            LEARNING_RATE)

        if CONTINUE:
            self.policy.load()

        # Episode policy and reward history
        self.policy_history = torch.zeros(MAX_EPISODE_LENGTH)
        self.current_episode_rewards = []

    def update_policy(self):
        R = 0
        rewards = []

        # Discount future rewards back to the present using gamma
        for reward in self.current_episode_rewards[::-1]:
            R = reward + DISCOUNT_FACTOR * R
            rewards.insert(0, R)

        # Scale rewards
        rewards = torch.tensor(rewards)
        if rewards.numel() > 1:
            rewards = (rewards - rewards.mean()) / (rewards.std() + FLOAT_EPS)

        # Calculate loss
        loss = torch.sum(self.policy_history[:len(rewards)] * rewards) * -1

        # Update network weights
        if loss != 0:
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        # Reset policy history and rewards
        self.policy_history = torch.zeros(MAX_EPISODE_LENGTH)
        self.current_episode_rewards = []

    def select_action(self, state):
        # Select an action (0 or 1) by running policy model and choosing based
        # on the probabilities in state
        action_probabilities = self.policy(state.cpu())
        action_distribution = Categorical(action_probabilities)
        action = action_distribution.sample()

        # Add log probability of our chosen action to our history
        self.policy_history[self.episode_length] = action_distribution.log_prob(action)  # nopep8

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
        self.current_episode_rewards.append(reward)


rospy.init_node('policy_gradient_training', anonymous=True)
node = PolicyGradientTrainingNode()
rospy.spin()
