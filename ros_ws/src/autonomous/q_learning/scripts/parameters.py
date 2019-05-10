import torch
import torch.nn as nn
import torch.nn.functional as F

import os
import rospy

# General parameters

ACTIONS = [(-0.5, 0.2), (0.5, 0.2)]
ACTION_COUNT = len(ACTIONS)

LASER_SAMPLE_COUNT = 16  # Only use some of the LIDAR measurements

UPDATE_FREQUENCY = 30

MODEL_FILENAME = "model.to"

class NeuralQEstimator(nn.Module):
    def __init__(self):
        super(NeuralQEstimator, self).__init__()
        self.fc1 = nn.Linear(LASER_SAMPLE_COUNT, 32)
        self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(32, ACTION_COUNT)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

    def load(self):
        if os.path.isfile(MODEL_FILENAME):
            self.load_state_dict(torch.load(MODEL_FILENAME))
            rospy.loginfo("Model parameters loaded.")
    
    def save(self):
        torch.save(self.state_dict(), MODEL_FILENAME)


# Training parameters

DISCOUNT_FACTOR = 0.99  # aka gamma

MAX_EPISODE_LENGTH = 300
# Sample neural net update from the replay memory. It contains this many episodes.
MEMORY_SIZE = 1000

BATCH_SIZE = 128
LEARNING_RATE = 0.001

# Probability to select a random episode starts at EPS_START
# and reaches EPS_END once EPS_DECAY episodes are completed.
EPS_START = 1.0
EPS_END = 0.1
EPS_DECAY = 5000