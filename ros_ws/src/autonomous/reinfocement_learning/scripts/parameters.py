import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

import os
import rospy

from rospkg import RosPack

# TOPICS
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_SCAN = "/scan"
TOPIC_CRASH = "/crash"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"
TOPIC_EPISODE_RESULT = "/qlearning/episode_result"

# General parameters

ACTIONS = [(-0.8, 0.1), (0.8, 0.1), (0.5, 0.2),
           (-0.5, 0.2), (0, 0.2), (0, 0.4)]
ACTION_COUNT = len(ACTIONS)

# Only use some of the LIDAR measurements
# When changing this value, also update laser_sample_count in qlearning.launch
LASER_SAMPLE_COUNT = 8


MODEL_FILENAME = os.path.join(RosPack().get_path("q_learning"), "model.to")


class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()

        self.fc1 = nn.Linear(LASER_SAMPLE_COUNT, 32)
        self.fc2 = nn.Linear(32, 18)
        self.fc3 = nn.Linear(18, ACTION_COUNT)

        # Episode policy and reward history
        self.policy_history = Variable(torch.Tensor())
        self.reward_episode = []

        # Overall reward and loss history
        self.reward_history = []
        self.loss_history = []

    def forward(self, x):
        model = torch.nn.Sequential(
            self.fc1,
            nn.ReLU(),
            self.fc2,
            nn.ReLU(),
            self.fc3,
            nn.Softmax(dim=-1)
        )
        return model(x)

    def load(self):
        if os.path.isfile(MODEL_FILENAME):
            self.load_state_dict(torch.load(MODEL_FILENAME))
            rospy.loginfo("Model parameters loaded.")

    def save(self):
        torch.save(self.state_dict(), MODEL_FILENAME)


class NeuralQEstimator(nn.Module):
    def __init__(self):
        super(NeuralQEstimator, self).__init__()
        self.fc1 = nn.Linear(LASER_SAMPLE_COUNT, 64)
        self.fc2 = nn.Linear(64, 32)
        self.fc3 = nn.Linear(32, ACTION_COUNT)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

    def load(self):
        self.load_state_dict(torch.load(MODEL_FILENAME))
        rospy.loginfo("Model parameters loaded.")

    def save(self):
        torch.save(self.state_dict(), MODEL_FILENAME)


# Training parameters

# Start by loading previously trained parameters.
# If this is False, training will start from scratch
USE_EXISTING_PARAMETERS = False

DISCOUNT_FACTOR = 0.99  # aka gamma

MAX_EPISODE_LENGTH = 5000
# Sample neural net update batch from the replay memory.
# It contains this many steps.
MEMORY_SIZE = 5000

BATCH_SIZE = 128
LEARNING_RATE = 0.0001
LEARNING_RATE_POLICY_GRADIENT = 0.001

# Probability to select a random episode starts at EPS_START
# and reaches EPS_END once EPS_DECAY episodes are completed.
EPS_START = 1.0
EPS_END = 0.3
EPS_DECAY = 10000
