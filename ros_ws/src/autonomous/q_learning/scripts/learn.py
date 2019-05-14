#!/usr/bin/env python
# coding: utf-8

import car
import math
import random
from collections import namedtuple, deque
import rospy
from std_msgs.msg import String, Empty

import torch
import torch.optim as optim
import torch.nn.functional as F

from simulation_tools.reset_car import Point
import simulation_tools.reset_car as reset_car

from simulation_tools.track import track

from parameters import *

import time

rospy.init_node('learn', anonymous=True)


def replay():
    global optimization_step_count
    if len(memory) < 500:
        return

    if optimization_step_count == 0:
        rospy.loginfo("Model optimization started.")

    transitions = random.sample(memory, min(BATCH_SIZE, len(memory)))

    states, actions, rewards, next_states, is_terminal = tuple(zip(*transitions))  # nopep8

    states = torch.stack(states)
    actions = torch.tensor(actions, device=device, dtype=torch.long)
    rewards = torch.tensor(rewards, device=device, dtype=torch.float)
    next_states = torch.stack(next_states)
    is_terminal = torch.tensor(is_terminal, device=device, dtype=torch.uint8)

    next_state_values = policy_net(next_states).max(1)[0].detach()
    q_updates = rewards + next_state_values * DISCOUNT_FACTOR
    q_updates[is_terminal] = rewards[is_terminal]

    optimizer.zero_grad()
    net_output = policy_net.forward(states)
    target = net_output.detach().clone()
    target[torch.arange(0, len(transitions), device=device,
                        dtype=torch.long), actions] = q_updates

    loss = F.smooth_l1_loss(net_output, target)
    loss.backward()
    for param in policy_net.parameters():
        param.grad.data.clamp_(-1, 1)
    optimizer.step()
    optimization_step_count += 1


def on_crash(_):
    global is_terminal_step
    if episode_length > 5:
        is_terminal_step = True


def get_eps_threshold():
    return EPS_END + (EPS_START - EPS_END) * \
        math.exp(-1. * total_step_count / EPS_DECAY)


def select_action(state):
    global net_output_debug_string

    if random.random() < get_eps_threshold():
        return random.randrange(ACTION_COUNT)

    with torch.no_grad():
        output = policy_net(state)
        if episode_length < 10:
            net_output_debug_string = ", ".join(
                ["{0:.1f}".format(v).rjust(5) for v in output.tolist()])
        return output.max(0)[1].item()


def perform_action(action_index):
    if action_index < 0 or action_index >= len(ACTIONS):
        raise Exception("Invalid action: " + str(action_index))
    car.drive(*ACTIONS[action_index])


def get_reward():
    track_position = track.localize(car.current_position)
    distance = abs(track_position.distance_to_center)

    if distance < 0.2:
        return 1
    elif distance < 0.4:
        return 0.7
    else:
        return 0.4


def log_progress():
    average_episode_length = sum(
        episode_length_history) / len(episode_length_history)
    average_cumulative_reward = sum(
        cumulative_reward_history) / len(cumulative_reward_history)
    rospy.loginfo("Episode " + str(episode_count) + ": "  # nopep8 \
        + str(episode_length).rjust(3) + " steps (" + str(int(average_episode_length)).rjust(3) + " avg), "  # nopep8 \
        + "return: " + "{0:.1f}".format(cumulative_reward).rjust(5)  # nopep8 \
        + " (" + "{0:.1f}".format(average_cumulative_reward).rjust(5) + " avg), "  # nopep8 \
        + ("memory: {0:d} / {1:d}, ".format(len(memory), MEMORY_SIZE) if len(memory) < MEMORY_SIZE else "")  # nopep8 \
        + "ε-greedy: " + str(int(get_eps_threshold() * 100)) + "% random, "  # nopep8 \
        + "nn opt steps: " + str(optimization_step_count) + ", "  # nopep8 \
        + "nn output: [" + net_output_debug_string + "], "  # nopep8 \
        + "sim time {0:.1f}x".format(real_time_factor)  # nopep8 \
        )  # nopep8


def on_complete_episode():
    global real_time_factor, episode_start_time, episode_count, episode_length, cumulative_reward
    episode_length_history.append(episode_length)
    cumulative_reward_history.append(cumulative_reward)

    now = time.time()
    real_time_factor = episode_length / (now - episode_start_time) / UPDATE_FREQUENCY  # nopep8
    episode_start_time = now
    log_progress()

    episode_count += 1
    episode_length = 0
    cumulative_reward = 0

    if episode_count % 50 == 0:
        policy_net.save()
        rospy.loginfo("Model parameters saved.")


reset_car.register_service()

rospy.loginfo("Initializing Pytorch...")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

policy_net = NeuralQEstimator().to(device)
if USE_EXISTING_PARAMETERS:
    policy_net.load()

optimizer = optim.Adam(policy_net.parameters(), lr=LEARNING_RATE)
memory = deque(maxlen=MEMORY_SIZE)

net_output_debug_string = ""

state = None
episode_count = 0
episode_length = 0
total_step_count = 0
optimization_step_count = 0
cumulative_reward = 0
is_terminal_step = False

episode_length_history = deque(maxlen=50)
cumulative_reward_history = deque(maxlen=50)

timer = rospy.Rate(UPDATE_FREQUENCY)
rospy.Subscriber("/crash", Empty, on_crash)

real_time_factor = 0
episode_start_time = time.time()

while not rospy.is_shutdown():
    if state is None:
        state = car.get_scan(LASER_SAMPLE_COUNT, device)
    action = select_action(state)
    perform_action(action)
    episode_length += 1
    total_step_count += 1

    timer.sleep()

    next_state = car.get_scan(LASER_SAMPLE_COUNT, device)
    reward = get_reward()
    cumulative_reward += reward
    memory.append((state, action, reward, next_state, is_terminal_step))
    state = next_state

    if is_terminal_step or episode_length >= MAX_EPISODE_LENGTH:
        reset_car.reset_random(
            max_angle=math.pi / 180 * 20,
            max_offset_from_center=0.2,
            forward=random.random() > 0.5)
        is_terminal_step = False
        state = None

        on_complete_episode()

        timer.sleep()

    replay()
