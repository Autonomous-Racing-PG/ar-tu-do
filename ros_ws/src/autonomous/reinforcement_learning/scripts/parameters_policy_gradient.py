import torch
import torch.nn as nn

import os
import rospy

from rospkg import RosPack

ACTIONS = [(-0.8, 0.1), (0.8, 0.1), (0.5, 0.2),
           (-0.5, 0.2), (0, 0.2), (0, 0.4)]
ACTION_COUNT = len(ACTIONS)

# Only use some of the LIDAR measurements
# When changing this value, also update laser_sample_count in
# policy_gradient.launch
LASER_SAMPLE_COUNT = 8

MODEL_FILENAME = os.path.join(RosPack().get_path(
    "reinforcement_learning"), "policy_gradient.to")

# Start by loading previously trained parameters.
# If this is False, training will start from scratch
CONTINUE = False

DISCOUNT_FACTOR = 0.99  # aka gamma

MAX_EPISODE_LENGTH = 5000

LEARNING_RATE = 0.001


class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()

        self.layers = nn.Sequential(
            nn.Linear(LASER_SAMPLE_COUNT, 32),
            nn.ReLU(),
            nn.Linear(32, 18),
            nn.ReLU(),
            nn.Linear(18, ACTION_COUNT),
            nn.Softmax(dim=-1)
        )

    def forward(self, x):
        return self.layers.forward(x)

    def load(self):
        if os.path.isfile(MODEL_FILENAME):
            self.load_state_dict(torch.load(MODEL_FILENAME))
            rospy.loginfo("Model parameters loaded.")

    def save(self):
        torch.save(self.state_dict(), MODEL_FILENAME)
