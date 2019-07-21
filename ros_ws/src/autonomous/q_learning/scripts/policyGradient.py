#!/usr/bin/env python
# coding: utf-8

from qlearning import QLearningNode, device
import random
import math
import time
from collections import deque
from std_msgs.msg import Empty
from parameters import *
import numpy as np

from torch.autograd import Variable
from torch.distributions import Categorical

import torch
import torch.optim as optim
import torch.nn.functional as F

from simulation_tools.reset_car import Point
import simulation_tools.reset_car as reset_car
from simulation_tools.track import track

from gazebo_msgs.msg import ModelStates

BATCH_INDICES = torch.arange(0, BATCH_SIZE, device=device, dtype=torch.long)

class QLearningTrainingNode(QLearningNode):
    def __init__(self):
        QLearningNode.__init__(self)
        
        self.policy1 = Policy()
        self.optimizer1 = optim.Adam(self.policy1.parameters(), lr=LEARNING_RATE)

        self.episode_count = 0
        self.episode_length = 0
        self.total_step_count = 0
        self.optimization_step_count = 0
        self.cumulative_reward = 0
        self.is_terminal_step = False

        self.net_output_debug_string = ""
        self.episode_length_history = deque(maxlen=50)
        self.cumulative_reward_history = deque(maxlen=50)

        self.state = None
        self.action = None
        self.car_position = None
        self.car_orientation = None

        self.drive_forward = None
        self.steps_with_wrong_orientation = 0

        self.real_time_factor = 0
        self.episode_start_time_real = time.time()
        self.episode_start_time_sim = rospy.Time.now().to_sec()

        if USE_EXISTING_PARAMETERS:
            self.policy1.load()

        self.optimizer = optim.Adam(self.policy.parameters(), lr=LEARNING_RATE)
        self.memory = deque(maxlen=MEMORY_SIZE)
        reset_car.register_service()

        rospy.Subscriber(TOPIC_CRASH, Empty, self.on_crash)
        rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, self.on_model_state_callback)  # nopep8

    def update_policy(self):
        R = 0
        rewards = []

        # Discount future rewards back to the present using gamma
        for r in self.policy1.reward_episode[::-1]:
            R = r + self.policy1.gamma * R
            rewards.insert(0,R)

        # Scale rewards
        rewards = torch.FloatTensor(rewards)
        if rewards.numel() > 1:
            rewards = (rewards - rewards.mean()) / (rewards.std() + np.finfo(np.float32).eps)

        # Calculate loss
        loss = (torch.sum(torch.mul(self.policy1.policy_history, Variable(rewards)).mul(-1), -1))

        # Update network weights
        if loss != 0:
            self.optimizer1.zero_grad()
            loss.backward()
            self.optimizer1.step()

            #Save episode history counters
            self.policy1.loss_history.append(loss.item())
            self.policy1.reward_history.append(np.sum(self.policy1.reward_episode))
        
        # Reset policy history and rewards
        self.policy1.policy_history = Variable(torch.Tensor())
        self.policy1.reward_episode= []

    def on_crash(self, _):
        global done
        if self.episode_length > 5:
            self.is_terminal_step = True
            done = True

    def select_action(self, state):
        #Select an action (0 or 1) by running policy model and choosing based on the probabilities in state
        state = self.state.type(torch.FloatTensor)
        state = self.policy1(Variable(state))
        c = Categorical(state)
        action = c.sample()

        # Add log probability of our chosen action to our history
        if self.policy1.policy_history.dim() != 0:
            self.policy1.policy_history = torch.cat([self.policy1.policy_history, c.log_prob(action).view(1)])
        else:
            self.policy1.policy_history = (c.log_prob(action))
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

        # The closer the car is to the middle of the race track, the more reward it gets
        if distance < 0.2:
            return 1 * scaleForSpeed
        elif distance < 0.4:
            return 0.7 * scaleForSpeed
        else:
            return -0.4 * scaleForSpeed
        
    def log_training_progress(self):
        average_episode_length = sum(
            self.episode_length_history) / len(self.episode_length_history)
        average_cumulative_reward = sum(
            self.cumulative_reward_history) / len(self.cumulative_reward_history)
        rospy.loginfo("Episode " + str(self.episode_count) + ": "  # nopep8 \
            + str(self.episode_length).rjust(3) + " steps (" + str(int(average_episode_length)).rjust(3) + " avg), "  # nopep8 \
            + "return: " + "{0:.1f}".format(self.cumulative_reward).rjust(5)  # nopep8 \
            + " (" + "{0:.1f}".format(average_cumulative_reward).rjust(5) + " avg), "  # nopep8 \
            + ("memory: {0:d} / {1:d}, ".format(len(self.memory), MEMORY_SIZE) if len(self.memory) < MEMORY_SIZE else "")  # nopep8 \
            + "time: {0:.1f}x, ".format(self.real_time_factor)  # nopep8 \
            + "laser: {0:.1f} Hz".format(float(self.episode_length) / (rospy.Time.now().to_sec() - self.episode_start_time_sim))  # nopep8 \
            )  # nopep8

    def on_complete_episode(self):
        self.update_policy()

        self.episode_length_history.append(self.episode_length)
        self.cumulative_reward_history.append(self.cumulative_reward)

        real_time = time.time()
        sim_time = rospy.Time.now().to_sec()
        self.real_time_factor = (sim_time - self.episode_start_time_sim) / (real_time - self.episode_start_time_real)  # nopep8
        self.log_training_progress()
        self.episode_start_time_real = real_time
        self.episode_start_time_sim = sim_time

        self.episode_count += 1
        self.episode_length = 0
        self.cumulative_reward = 0

        if self.episode_count % 50 == 0:
            self.policy1.save()
            rospy.loginfo("Model parameters saved.")

    def check_car_orientation(self):
        if self.car_position is None:
            return

        track_position = track.localize(self.car_position)
        car_direction = track_position.faces_forward(self.car_orientation)
        if car_direction != self.drive_forward:
            self.steps_with_wrong_orientation += 1
        else:
            self.steps_with_wrong_orientation = 0

        if self.steps_with_wrong_orientation > 2:
            self.is_terminal_step = True

    # Do an episode step every time laser scans are recieved
    def on_receive_laser_scan(self, message):
        state = self.convert_laser_message_to_tensor(message)

        if self.state is not None:
            self.check_car_orientation()
            reward = self.get_reward()
            self.cumulative_reward += reward
            self.policy1.reward_episode.append(reward)
            self.memory.append((self.state, self.action, reward, state, self.is_terminal_step))  # nopep8

        if self.is_terminal_step or self.episode_length >= MAX_EPISODE_LENGTH:
            self.drive_forward = random.random() > 0.5
            reset_car.reset_random(
                max_angle=math.pi / 180 * 20,
                max_offset_from_center=0.2,
                forward=self.drive_forward)
            self.is_terminal_step = False
            self.state = None
            if self.episode_length != 0:
                self.on_complete_episode()
        else:
            self.state = state
            self.action = self.select_action(state)
            self.perform_action(self.action)
            self.episode_length += 1
            self.total_step_count += 1

    def on_model_state_callback(self, message):
        if len(message.pose) < 2:
            return
        self.car_position = Point(
            message.pose[1].position.x,
            message.pose[1].position.y)
        self.car_orientation = message.pose[1].orientation


rospy.init_node('q_learning_training', anonymous=True)
node = QLearningTrainingNode()
rospy.spin()