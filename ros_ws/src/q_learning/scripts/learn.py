#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from tf.transformations import quaternion_from_euler

import math
import random
import numpy as np
from itertools import count
from collections import namedtuple
import threading
import time

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

ACTIONS = [(-0.7, 0.2), (-0.3, 0.3), (0, 0.4), (0.3, 0.3), (0.7, 0.2), (0.0, 0.6)]
ACTION_COUNT = len(ACTIONS)

LASER_SAMPLE_COUNT = 32 # Only use some of the LIDAR measurements
MAX_RANGE = 10 # Clamp lidar measurements to this distance

DISCOUNT_FACTOR = 0.999 # aka gamma

MAX_EPISODE_LENGTH = 500
BATCH_SIZE = 200 # Number of steps that are randomly sampled for each neural net update
MEMORY_SIZE = 4000 # Sample neural net update from the memory. It contains this many episodes.

# Probability to select a random episode starts at EPS_START
# and reaches EPS_END once EPS_DECAY episodes are completed.
EPS_START = 0.9
EPS_END = 0.05
EPS_DECAY = 800

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))
Episode = namedtuple("Episode", ("states", "actions", "rewards"))

class ReplayMemory(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if not self.full():
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def full(self):
        return len(self.memory) >= self.capacity

    def size(self):
        return len(self.memory)


class DQN(nn.Module):
    def __init__(self):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(LASER_SAMPLE_COUNT, 8)
        self.fc2 = nn.Linear(8, 8)
        self.fc3 = nn.Linear(8, ACTION_COUNT)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

def get_eps_threshold():
    return EPS_END + (EPS_START - EPS_END) * math.exp(-1. * total_steps / EPS_DECAY)

def select_action(state):
    if random.random() < get_eps_threshold():
        return random.randrange(ACTION_COUNT)
    else:
        with torch.no_grad():
            net_output = policy_net(state)
            return net_output.max(0)[1].item()

optimization_steps = 0

def optimize_model():
    while not rospy.is_shutdown():
        sample_size = int(memory.size() * 0.4)
        if (sample_size < 100):
            time.sleep(1)
            continue
        
        transitions = memory.sample(sample_size)
        
        global started_optimization
        if not started_optimization:
            started_optimization = True
            rospy.loginfo("Model optimization started.")

        # create a single Transition tuple with lists of states, actions, rewards and next states
        batch = Transition(*zip(*transitions))
        
        state_batch = torch.stack(batch.state)
        next_states = torch.stack(batch.next_state)
        action_batch = torch.tensor(batch.action, device=device, dtype=torch.long)
        action_batch = torch.unsqueeze(action_batch, 1)
        reward_batch = torch.cat(batch.reward)

        # Compute Q values for the actions that were taken
        net_output = policy_net(state_batch)
        state_action_values = net_output.gather(1, action_batch)
        next_state_values = target_net(next_states).max(1)[0].detach()

        # Compute the expected Q values
        expected_state_action_values = (next_state_values * DISCOUNT_FACTOR) + reward_batch

        # Compute Huber loss
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

        # Optimize the model
        optimizer.zero_grad()
        loss.backward()
        for param in policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        optimizer.step()
        global optimization_steps
        optimization_steps += 1

def respawn():
    state = ModelState()
    state.model_name = "racer"
    state.pose.position.x = 0
    state.pose.position.y = -0.5
    state.pose.position.z = 0

    q = quaternion_from_euler(2 * math.pi * random.random(), math.pi, 0)
    state.pose.orientation.x = q[0]
    state.pose.orientation.z = q[1]
    state.pose.orientation.w = q[2]
    state.pose.orientation.y = q[3]
    
    set_model_state(state)

def get_distance_travelled():
    if current_position is None or previous_position is None:
        return 0
    return ((current_position.x - previous_position.x)**2 + (current_position.y - previous_position.y)**2)**0.5

def crash_callback(_):
    distance_travelled = get_distance_travelled()
    if current_episode is None or len(current_episode.states) < 2:
        return
    reset_episode()

def log_progress():
    rospy.loginfo("Episode " + str(episode_count) + ": " \
        + str(len(current_episode.actions)) + " steps" \
        + ", cumulative reward: " + format(sum(current_episode.rewards).data[0].item(), ".2f") \
        + (", memory size: " + str(memory.size()) + " / " + str(memory.capacity) if not memory.full() else "") \
        + ", selecting " + str(int(get_eps_threshold() * 100)) + "% random actions")

def reset_episode():
    global previous_position, current_episode, episode_count

    if current_episode is not None:
        log_progress()

    current_episode = Episode([], [], [])
    episode_count += 1
    previous_position = None
    respawn()

def model_state_callback(message):
    global current_position, previous_position
    if len(message.pose) < 2:
        return
    current_position = message.pose[1].position

def publish_drive_parameters(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)

def drive(action_index):
    if action_index < 0 or action_index >= len(ACTIONS):
        raise Exception("Invalid action: " + str(action_index))
    publish_drive_parameters(*ACTIONS[action_index])

def get_random_action():
    return randint(0, len(ACTIONS) - 1)

def laser_callback(message):
    global laser_scan
    laser_scan = message

def get_state():
    if laser_scan == None:
        return torch.tensor([0 for _ in range(LASER_SAMPLE_COUNT)], device=device, dtype=torch.float32)
    count = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment
    indices = [int(i * count / LASER_SAMPLE_COUNT) for i in range(LASER_SAMPLE_COUNT)]
    values = [min(1, laser_scan.ranges[i] / MAX_RANGE) ** 0.5 for i in indices]
    return torch.tensor(values, device=device)

rospy.init_node('learn', anonymous=True)

rospy.loginfo("Initializing Pytorch...")
policy_net = DQN().to(device)
target_net = DQN().to(device)
target_net.load_state_dict(policy_net.state_dict())
target_net.eval()

optimizer = optim.RMSprop(policy_net.parameters())
memory = ReplayMemory(MEMORY_SIZE)

current_position = None
previous_position = None
laser_scan = None

current_episode = None
episode_count = 0
total_steps = 0
started_optimization = False

rospy.loginfo("Waiting for gazebo...")
rospy.wait_for_service('/gazebo/set_model_state')
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

rospy.Subscriber("/crash", Empty, crash_callback)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/scan", LaserScan, laser_callback)
drive_parameters_publisher = rospy.Publisher("/commands/drive_param", drive_param, queue_size=1)
reset_episode()
rate = rospy.Rate(40)

rospy.loginfo("Started training.")

threading.Thread(target=optimize_model).start()

while not rospy.is_shutdown():
    if len(current_episode.actions) > MAX_EPISODE_LENGTH:
        reset_episode()
    else:
        state = get_state()
        reward = get_distance_travelled()
        action = select_action(state)
        
        drive(action)
        previous_position = current_position

        if len(current_episode.states) > 0:
            reward = torch.tensor([reward], device=device, dtype=torch.float32)
            try:
                memory.push(current_episode.states[-1], action, state, reward)
            except IndexError:
                # Race condition. The car crashed and the current episode is empty. Do nothing.
                pass


        current_episode.states.append(state)
        current_episode.actions.append(action)
        current_episode.rewards.append(reward)
        if memory.full():
            total_steps += 1

    rate.sleep()
