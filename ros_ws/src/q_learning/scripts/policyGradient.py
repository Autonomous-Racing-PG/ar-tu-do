#!/usr/bin/env python

import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.autograd import Variable
from torch.distributions import Categorical

import math
import random
import rospy
import car

rospy.init_node('learn', anonymous=True)


ACTIONS = [(-0.4, 0.1),(0.4, 0.1)]
#ACTION_COUNT = len(ACTIONS)

# angle space are 18 possible steering directions from -1 to 1
#angle_space = np.linspace(start=-1, stop=1, num=18, endpoint=True)
#angle_space_len = len(angle_space)
#angle_space = np.insert(angle_space, (angle_space_len/2), 0.0)

# velocity space are the possible speed settings for the different steering directions
# with higher speeds for smaller angles
#velocity_space_half = np.concatenate((
#        np.repeat(0.05, 6),
#        np.repeat(0.1, 2),
#        np.repeat(0.15, 1)))
#velocity_space = np.concatenate(
#    (velocity_space_half,
#    [2.0],
#    np.flip(velocity_space_half)))
   
#ACTIONS = zip(angle_space, velocity_space)
ACTION_COUNT = len(ACTIONS)

#print(ACTION_COUNT)
#print(ACTIONS)

LASER_SAMPLE_COUNT = 32  # Only use some of the LIDAR measurements

learning_rate = 0.01
gamma = 0.99
running_reward = 0
done = False
crash = False
deadlock = 0

class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.state_space = LASER_SAMPLE_COUNT
        self.action_space = ACTION_COUNT
        
        self.l1 = nn.Linear(self.state_space, 64, bias=False)
        self.l2 = nn.Linear(64, self.action_space, bias=False)
        
        self.gamma = gamma
        
        # Episode policy and reward history 
        self.policy_history = Variable(torch.Tensor()) 
        self.reward_episode = []
        # Overall reward and loss history
        self.reward_history = []
        self.loss_history = []

    def forward(self, x):    
        model = torch.nn.Sequential(
            self.l1,
            nn.Dropout(p=0.6),
            nn.ReLU(),
            self.l2,
            nn.Softmax(dim=-1)
        )
        return model(x)

policy = Policy()
optimizer = optim.Adam(policy.parameters(), lr=learning_rate)

def select_action(state):
    #Select an action (0 or 1) by running policy model and choosing based on the probabilities in state
    state = state.type(torch.FloatTensor)
    state = policy(Variable(state))
    
    c = Categorical(state)
    
    action = c.sample()
    
    # Add log probability of our chosen action to our history    
    if policy.policy_history.dim() != 0:
        policy.policy_history = torch.cat([policy.policy_history, c.log_prob(action).view(1)])
    else:
        policy.policy_history = (c.log_prob(action))
    return action

def update_policy():
    R = 0
    rewards = []
    
    # Discount future rewards back to the present using gamma
    for r in policy.reward_episode[::-1]:
        R = r + policy.gamma * R
        rewards.insert(0,R)
        
    # Scale rewards
    rewards = torch.FloatTensor(rewards)
    
    ##########DEBUG
    #rospy.loginfo("BEFORE rewards: " + str(rewards))
    rewards1 = rewards - rewards.mean()
    #rospy.loginfo("rewards1: " + str(rewards1))
    rewards2 = rewards.std() + np.finfo(np.float32).eps
    rospy.loginfo("rewards std: " + str(rewards.std()))
    rospy.loginfo("rewards finfo: " + str(np.finfo(np.float32).eps))
    
    if rewards.numel() > 1:
        rewards = (rewards - rewards.mean()) / (rewards.std() + np.finfo(np.float32).eps)

    #print('REWARDS: {}'.format(rewards))
    
    #rospy.loginfo("AFTER rewards: " + str(rewards))
    
    # Calculate loss
    loss = (torch.sum(torch.mul(policy.policy_history, Variable(rewards)).mul(-1), -1))

    print('LOSS: {}'.format(loss))
    
    # Update network weights
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    
    #Save and intialize episode history counters
    policy.loss_history.append(loss.item())
    policy.reward_history.append(np.sum(policy.reward_episode))
    policy.policy_history = Variable(torch.Tensor())
    policy.reward_episode= []

def perform_action(action_index):
    if action_index < 0 or action_index >= len(ACTIONS):
        raise Exception("Invalid action: " + str(action_index))
    car.drive(*ACTIONS[action_index])

def on_crash():
    global crash
    crash = True

## TODO REWARDS besser skalieren
## Maybe zu viele actions sodass zu kompliziert

def main(episodes, device):
    global done, crash, deadlock 
    running_reward = 1
    average_reward_per_episode = 0
    for episode in range(episodes):

        # Spawns the car the coordinates (x=0, y= -0.5) with an orientation of math.pi + (0 if random.random() > 0.5 else math.pi)
        car.reset((0, -0.5), math.pi + (0 if random.random() > 0.5 else math.pi))
        state = car.get_scan(LASER_SAMPLE_COUNT, device)
        
        # If true, the current episode will end instantly
            

        for time in range(400000):
            done = False  
            crash = False
            # Choose an action w.r.t the current state
            action = select_action(state)
            # Step through environment using chosen action
            perform_action(action)

            # Get the next state
            state = car.get_scan(LASER_SAMPLE_COUNT, device)
            # The faster the car is driving, the bigger the reward
            running_reward = 10 * ACTIONS[action][1]
            
            # True when the car intersects with a wall 
            if crash:
                running_reward = -10000 * ACTIONS[action][1] * 2000/(time+1)
                done = True

            # True if car is flipped 
            car_orientation = car.get_car_pos_x()
            if (car_orientation[1] < -0.2 and car_orientation[1] > -0.322):
                running_reward = -3000
                done = True

            # End the episode if the car is driving against a wall for more than 4 steps
            if running_reward < 0:
                deadlock += 1
                if deadlock >= 5:
                    done = True
                    deadlock = 0
            else:
                deadlock = 0
            
            # Save reward
            policy.reward_episode.append(running_reward)
            average_reward_per_episode += running_reward
            # End the episode
            if done:
                break
        
        # Used to determine when the environment is solved.
        #running_reward = (running_reward * 0.99) + (time * 0.01)
        if(time == 0):
            policy.reward_episode = []
            policy.policy_history = Variable(torch.Tensor())
        else:
            update_policy()

        print('Episode {}\tLast length: {:5d}\tSum Reward: {}'.format(episode, time, average_reward_per_episode))
        average_reward_per_episode = 0

rospy.loginfo("Initializing Pytorch...")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

car.register_crash_callback(on_crash)
episodes = 1000
main(episodes, device)

