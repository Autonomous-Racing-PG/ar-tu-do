#!/usr/bin/env python

import car
import math
import random
import numpy as np
from itertools import count
from collections import namedtuple
import threading
import time
import rospy

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

from ring_buffer import RingBuffer

rospy.init_node('learn', anonymous=True)

ACTIONS = [(-0.4, 0.2), (0.4, 0.2)]
ACTION_COUNT = len(ACTIONS)

LASER_SAMPLE_COUNT = 64  # Only use some of the LIDAR measurements

DISCOUNT_FACTOR = 0.999  # aka gamma

MAX_EPISODE_LENGTH = 600
# Sample neural net update from the memory. It contains this many episodes.
MEMORY_SIZE = 10000

BATCH_SIZE = 200
LEARNING_RATE = 0.002

# Probability to select a random episode starts at EPS_START
# and reaches EPS_END once EPS_DECAY episodes are completed.
EPS_START = 0.9
EPS_END = 0.10
EPS_DECAY = 500000
REWARD = 2
ACC_REWARD = 0
TOTAL_REWARD = 0

class NeuralQEstimator(nn.Module):
    def __init__(self):
        super(NeuralQEstimator, self).__init__()
        self.fc1 = nn.Linear(LASER_SAMPLE_COUNT, 32)
        self.fc2 = nn.Linear(32, 16)
        self.fc3 = nn.Linear(16, ACTION_COUNT)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)


'''
Sample some states in which the car crashed and learn
that the Q value for these states should always be 0
'''


def learn_crash_states():
    if len(crash_states) < 10:
        return
    optimizer.zero_grad()
    states = random.sample(crash_states, min(100, len(crash_states)))

    state_batch = torch.stack(states)
    q = torch.zeros(len(states), ACTION_COUNT, device=device)

    # Compute Q values for the actions that were taken
    policy_net_output = policy_net(state_batch)

    # Compute Huber loss
    loss = F.smooth_l1_loss(policy_net_output, q)

    # Optimize the model
    loss.backward()
    optimizer.step()


def optimize_model():
    global optimization_step_count
    while not rospy.is_shutdown():
        optimizer.zero_grad()
        if (100 > memory.size()):
            time.sleep(1)
            continue

        if optimization_step_count == 0:
            rospy.loginfo("Model optimization started.")

        transitions = memory.sample(min(BATCH_SIZE, memory.size()))

        states, actions, next_states, rewards = tuple(zip(*transitions))

        state_batch = torch.stack(states)
        next_state_batch = torch.stack(next_states)
        action_batch = torch.tensor(actions, device=device, dtype=torch.long)
        reward_batch = torch.tensor(rewards, device=device, dtype=torch.float)

        # Compute Q values for the actions that were taken
        policy_net_output = policy_net(state_batch)
        state_action_values = policy_net_output
        target_net_output = target_net(next_state_batch)
        next_state_values = target_net_output.max(1)[0].detach()

        # Compute the expected Q values
        expected_state_action_values = (
            next_state_values * DISCOUNT_FACTOR) + reward_batch
        target_net_output[torch.arange(0,
                                       target_net_output.size()[0],
                                       device=device,
                                       dtype=torch.long),
                          action_batch] = expected_state_action_values

        # Compute Huber loss
        loss = F.mse_loss(state_action_values, target_net_output.detach())

        # Optimize the model
        loss.backward()
        for param in policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        optimizer.step()
        optimization_step_count += 1

        learn_crash_states()

        if optimization_step_count % 10 == 0:
            target_net.load_state_dict(policy_net.state_dict())


def on_crash():
    global REWARD
    if current_episode_states is None or len(current_episode_states) < 2:
        return
    if len(current_episode_states) > 10:
        crash_states.append(current_episode_states[-1])
    REWARD = -10 

def log_progress():
    global TOTAL_REWARD
    test = policy_net.forward(TEST_SCAN).tolist()
    rospy.loginfo("Episode " + str(episode_count) + ": "  # nopep8 \
        + str(len(current_episode_states)) + " steps"  # nopep8 \
        + (", memory size: " + str(memory.size()) + " / " + str(memory.capacity) if not memory.full() else "")  # nopep8 \
        + ", selecting " + str(int(get_eps_threshold() * 100)) + "% random actions"  # nopep8 \
        + ", optimization steps: " + str(optimization_step_count) # nopep8 \
        + ", test: " + str(test))  # nopep8
    if episode_count % 10 == 0:
        rospy.loginfo("Average accumulated Reward: " + str(TOTAL_REWARD/episode_count))


def reset_episode():
    global current_episode_states, episode_count

    if current_episode_states is not None:
        log_progress()

    current_episode_states = []
    episode_count += 1
    car.reset((10, 1), math.pi * 0.25 + (0 if random.random() > 0.5 else math.pi))


def get_eps_threshold():
    return EPS_END + (EPS_START - EPS_END) * \
        math.exp(-1. * total_step_count / EPS_DECAY)


def select_action(state):
    # Calculate threshold for epsilon greedy strategy
    if random.random() < get_eps_threshold():
        return random.randrange(ACTION_COUNT)
    else:
        with torch.no_grad():
            net_output = policy_net(state)
            return net_output.max(0)[1].item()


def perform_action(action_index):
    if action_index < 0 or action_index >= len(ACTIONS):
        raise Exception("Invalid action: " + str(action_index))
    car.drive(*ACTIONS[action_index])


def step():
    global REWARD, ACC_REWARD, TOTAL_REWARD
    car_flipped = False
    car_orientation = car.get_car_pos_x() 
    if len(current_episode_states) > MAX_EPISODE_LENGTH:
        rospy.loginfo("Accumulated Reward:" + str(ACC_REWARD))
        TOTAL_REWARD += ACC_REWARD
        ACC_REWARD = 0
        reset_episode()
        return

    if car_orientation[1] < -0.2 and car_orientation > -0.322:
        rospy.loginfo("CAR FLIPPED")
        REWARD = -6000
        car_flipped = True

    ACC_REWARD += REWARD
    state = car.get_scan(LASER_SAMPLE_COUNT, device)
    
    action = select_action(state)

    perform_action(action)

    if len(current_episode_states) > 0:
        try:
            transistion = (current_episode_states[-1], action, state, REWARD)
            memory.push(transistion)
        except IndexError:
            # Race condition. The car crashed and the current episode is empty.
            # Do nothing.
            pass

    current_episode_states.append(state)
    global total_step_count
    total_step_count += 1
    REWARD = 2
    
    if car_flipped:
        rospy.loginfo("Accumulated Reward:" + str(ACC_REWARD))
        TOTAL_REWARD += ACC_REWARD
        ACC_REWARD = 0
        reset_episode()
        return

    #(-2.5115459480578326, -0.20815514356980613, -1.3759510978008045)
    #(-3.141217254439627, -0.32171944078397496, -1.8755338157994788)
    #(-3.141274954812683, -0.3217224998170558, -1.7335801543992733)


rospy.loginfo("Initializing Pytorch...")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
policy_net = NeuralQEstimator().to(device)
target_net = NeuralQEstimator().to(device)
target_net.load_state_dict(policy_net.state_dict())
target_net.eval()

TEST_SCAN = torch.tensor([ 3.7452,  3.5563,  3.4159,  3.3219,  3.2454,  3.2170,  3.2289,  3.2187,
    0.2295,  0.2486,  0.2640,  0.2501,  0.2589,  0.2591,  0.2718,  0.2589,
    0.2721,  0.2887,  0.2888,  0.3232,  0.3380,  0.3530,  0.3963,  0.4139,
    0.4555,  0.5164,  0.6194,  0.7361,  0.9580,  1.2119,  1.3391, 12.3717,
    6.3367,  5.2729,  4.6968,  4.1166,  3.5303,  2.9208,  2.4639,  2.1669,
    1.7586,  1.6287,  1.6486,  1.5390,  1.4283,  1.3593,  1.3124,  1.2491,
    1.2228,  1.1751,  1.1655,  1.1324,  1.1337,  1.1198,  1.1232,  1.1507,
    1.1621,  1.1873,  1.2166,  1.2648,  1.3209,  1.3559,  1.4453,  1.5478],
device=device)

optimizer = optim.SGD(policy_net.parameters(), lr=LEARNING_RATE)
memory = RingBuffer(MEMORY_SIZE)

current_episode_states = None
crash_states = []
episode_count = 0
total_step_count = 0
optimization_step_count = 0

reset_episode()
timer = rospy.Rate(20)
car.register_crash_callback(on_crash)

rospy.loginfo("Started driving.")

threading.Thread(target=optimize_model).start()

while not rospy.is_shutdown():
    step()
    timer.sleep()
