#!/usr/bin/env python
# coding: utf-8

from qlearning import QLearningNode, device
import random
import math
import time
from collections import deque
from std_msgs.msg import Empty
from parameters import *

import torch
import torch.optim as optim
import torch.nn.functional as F

from simulation_tools.reset_car import Point
import simulation_tools.reset_car as reset_car
from simulation_tools.track import track

from gazebo_msgs.msg import ModelStates
from q_learning.msg import EpisodeResult

BATCH_INDICES = torch.arange(0, BATCH_SIZE, device=device, dtype=torch.long)


class QLearningTrainingNode(QLearningNode):
    def __init__(self):
        QLearningNode.__init__(self)

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
            self.policy.load()

        self.optimizer = optim.Adam(self.policy.parameters(), lr=LEARNING_RATE)
        self.memory = deque(maxlen=MEMORY_SIZE)
        reset_car.register_service()

        rospy.Subscriber(TOPIC_CRASH, Empty, self.on_crash)
        rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, self.on_model_state_callback)  # nopep8

        self.episode_result_publisher = rospy.Publisher(
            TOPIC_EPISODE_RESULT, EpisodeResult, queue_size=1)

    def replay(self):
        if len(self.memory) < 500 or len(self.memory) < BATCH_SIZE:
            return

        if self.optimization_step_count == 0:
            rospy.loginfo("Model optimization started.")

        transitions = random.sample(self.memory, BATCH_SIZE)  # nopep8
        states, actions, rewards, next_states, is_terminal = tuple(zip(*transitions))  # nopep8

        states = torch.stack(states)
        actions = torch.tensor(actions, device=device, dtype=torch.long)
        rewards = torch.tensor(rewards, device=device, dtype=torch.float)
        next_states = torch.stack(next_states)
        is_terminal = torch.tensor(
            is_terminal, device=device, dtype=torch.uint8)

        next_state_values = self.policy.forward(next_states).max(1)[0].detach()
        q_updates = rewards + next_state_values * DISCOUNT_FACTOR
        q_updates[is_terminal] = rewards[is_terminal]

        self.optimizer.zero_grad()
        net_output = self.policy.forward(states)
        loss = F.smooth_l1_loss(net_output[BATCH_INDICES, actions], q_updates)
        loss.backward()
        for param in self.policy.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()
        self.optimization_step_count += 1

    def on_crash(self, _):
        if self.episode_length > 5:
            self.is_terminal_step = True

    def get_epsilon_greedy_threshold(self):
        return EPS_END + (EPS_START - EPS_END) * \
            math.exp(-1. * self.total_step_count / EPS_DECAY)

    def select_action(self, state):
        use_epsilon_greedy = self.episode_count % 2 == 0
        if use_epsilon_greedy and random.random() < self.get_epsilon_greedy_threshold():
            return random.randrange(ACTION_COUNT)

        with torch.no_grad():
            output = self.policy(state)
            if self.episode_length < 10:
                self.net_output_debug_string = ", ".join(
                    ["{0:.1f}".format(v).rjust(5) for v in output.tolist()])
            return output.max(0)[1].item()

    def get_reward(self):
        track_position = track.localize(self.car_position)
        distance = abs(track_position.distance_to_center)

        if distance < 0.2:
            return 1
        elif distance < 0.4:
            return 0.7
        else:
            return 0.4

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
            + "ε-greedy: " + str(int(self.get_epsilon_greedy_threshold() * 100)) + "% random, "  # nopep8 \
            + "replays: " + str(self.optimization_step_count) + ", "  # nopep8 \
            + "q: [" + self.net_output_debug_string + "], "  # nopep8 \
            + "time: {0:.1f}x, ".format(self.real_time_factor)  # nopep8 \
            + "laser: {0:.1f} Hz".format(float(self.episode_length) / (rospy.Time.now().to_sec() - self.episode_start_time_sim))  # nopep8 \
            )  # nopep8

    def on_complete_episode(self):
        self.episode_length_history.append(self.episode_length)
        self.cumulative_reward_history.append(self.cumulative_reward)

        self.episode_result_publisher.publish(
            reward=self.cumulative_reward, length=int(self.episode_length))

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
            self.policy.save()
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

    def on_receive_laser_scan(self, message):
        state = self.convert_laser_message_to_tensor(message)

        if self.state is not None:
            self.check_car_orientation()
            reward = self.get_reward()
            self.cumulative_reward += reward
            self.memory.append((self.state, self.action, reward, state, self.is_terminal_step))  # nopep8

        if self.is_terminal_step or self.episode_length >= MAX_EPISODE_LENGTH:
            self.drive_forward = random.random() > 0.5
            reset_car.reset_random(
                max_angle=math.pi / 180 * 20,
                max_offset_from_center=0.2,
                forward=self.drive_forward)
            self.is_terminal_step = False
            self.state = None
            self.on_complete_episode()
        else:
            self.state = state
            self.action = self.select_action(state)
            self.perform_action(self.action)
            self.episode_length += 1
            self.total_step_count += 1

        self.replay()

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
