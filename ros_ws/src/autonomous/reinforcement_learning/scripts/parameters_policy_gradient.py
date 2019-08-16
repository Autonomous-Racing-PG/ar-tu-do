import torch
import torch.nn as nn
from torch.autograd import Variable

import os
import rospy

from rospkg import RosPack

ACTIONS = [(-0.8, 0.1), (0.8, 0.1), (0.5, 0.2), (-0.5, 0.2), (0, 0.2), (0, 0.4)]
ACTION_COUNT = len(ACTIONS)

# Only use some of the LIDAR measurements
# When changing this value, also update laser_sample_count in qlearning.launch
LASER_SAMPLE_COUNT = 8

MODEL_FILENAME = os.path.join(RosPack().get_path(
    "reinforcement_learning"), "policy_gradient.to")

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


# Start by loading previously trained parameters.
# If this is False, training will start from scratch
CONTINUE = False

DISCOUNT_FACTOR = 0.99  # aka gamma

MAX_EPISODE_LENGTH = 5000

LEARNING_RATE = 0.001
