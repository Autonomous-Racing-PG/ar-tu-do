#!/usr/bin/env python

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

rospy.init_node('learn', anonymous=True)

def replay():
    global optimization_step_count
    if len(memory) < 100:
        return

    if optimization_step_count == 0:
        rospy.loginfo("Model optimization started.")

    transitions = random.sample(memory, min(BATCH_SIZE, len(memory)))

    states, actions, rewards, next_states, is_terminal = tuple(zip(*transitions))

    state_batch = torch.stack(states)
    action_batch = torch.tensor(actions, device=device, dtype=torch.long)
    reward_batch = torch.tensor(rewards, device=device, dtype=torch.float)
    next_state_batch = torch.stack(next_states)
    is_terminal_batch = torch.tensor(is_terminal, device = device, dtype = torch.long)
        
    next_state_values = policy_net(next_state_batch).max(1)[0].detach()
    q_updates = reward_batch + next_state_values * DISCOUNT_FACTOR
    q_updates[is_terminal_batch] = reward_batch[is_terminal_batch]

    optimizer.zero_grad()
    net_output = policy_net.forward(state_batch)
    target = net_output.detach().clone()
    target[torch.arange(0, len(transitions), device=device, dtype=torch.long), action_batch] = q_updates
    
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
    # Calculate threshold for epsilon greedy strategy
    if random.random() < get_eps_threshold():
        return random.randrange(ACTION_COUNT)
    else:
        with torch.no_grad():
            net_output = policy_net(state)
            if episode_length < 10:
                net_output_debug_string = str(net_output)
            return net_output.max(0)[1].item()


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
    rospy.loginfo("Episode " + str(episode_count) + ": "  # nopep8 \
        + str(episode_length).rjust(4) + " steps"  # nopep8 \
        + (", memory size: " + str(len(memory)) + " / " + str(MEMORY_SIZE) if len(memory) < MEMORY_SIZE else "")  # nopep8 \
        + ", selecting " + str(int(get_eps_threshold() * 100)) + "% random actions"  # nopep8 \
        + ", optimization steps: " + str(optimization_step_count) + " " + net_output_debug_string)  # nopep8


reset_car.register_service()

rospy.loginfo("Initializing Pytorch...")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
policy_net = NeuralQEstimator().to(device)
policy_net.load()

optimizer = optim.Adam(policy_net.parameters(), lr=LEARNING_RATE)
memory = deque(maxlen=MEMORY_SIZE)

net_output_debug_string = ""

state = None
episode_count = 0
episode_length = 0
total_step_count = 0
optimization_step_count = 0
is_terminal_step = False

timer = rospy.Rate(UPDATE_FREQUENCY)
rospy.Subscriber("/crash", Empty, on_crash)
rospy.loginfo("Started driving.")

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
    memory.append((state, action, reward, next_state, is_terminal_step))
    state = next_state

    if is_terminal_step or episode_length >= MAX_EPISODE_LENGTH:
        reset_car.reset_random(max_angle=math.pi / 180 * 20, max_offset_from_center=0.2)
        is_terminal_step = False
        log_progress()

        state = None
        episode_count += 1
        episode_length = 0

        if episode_count % 50 == 0:
            policy_net.save()
            rospy.loginfo("Model parameters saved.")

        timer.sleep()

    replay()
