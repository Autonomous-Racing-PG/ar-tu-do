import torch
import torch.nn as nn
import torch.nn.functional as F

import os
import rospy
import numpy as np

from rospkg import RosPack

# TOPICS
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_SCAN = "/scan"
TOPIC_CRASH = "/crash"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"

# General parameters

ACTIONS = [(1, 0.2), (0.5, 0.25), (0.25, 0.3), (0.0, 0.35)]
ACTIONS += [(-steering, throttle) for steering, throttle in ACTIONS if steering != 0]
ACTION_COUNT = len(ACTIONS)


print(ACTION_COUNT)
print(ACTIONS)

# Only use some of the LIDAR measurements
# When changing this value, also update laser_sample_count in qlearning.launch
LASER_SAMPLE_COUNT = 1080

MODEL_FILENAME = os.path.join(RosPack().get_path("q_learning"), "model.to")


class NeuralQEstimator(nn.Module):
    def __init__(self):
        super(NeuralQEstimator, self).__init__()

        # (N, 2, 1080)
        self.cnv1 = nn.Conv1d(in_channels=2, out_channels=8, kernel_size=5)
        # (N, 8, 1076)
        self.mp1 = nn.MaxPool1d(kernel_size=4)
        # (N, 8, 269)

        self.cnv2 = nn.Conv1d(in_channels=8, out_channels=16, kernel_size=5)
        # (N, 16, 265)
        self.mp2 = nn.MaxPool1d(kernel_size=4)
        # (N, 16, 66)

        self.cnv3 = nn.Conv1d(in_channels=16, out_channels=32, kernel_size=5)
        # (N, 32, 62)
        self.mp3 = nn.MaxPool1d(kernel_size=4)
        # (N, 32, 15)

        # reshape
        # (N, 32 * 15)

        #self.fc = nn.Linear(32*15, ACTION_COUNT)
        # (N, ACTION_COUNT)

        self.fc1 = nn.Linear(32*15, 240)
        self.drp1 = nn.Dropout(p=0.5)
        self.fc2 = nn.Linear(240, 120)
        self.drp2 = nn.Dropout(p=0.5)
        self.fc3 = nn.Linear(120, ACTION_COUNT)

    def forward(self, x):

        # If processing a single sample, turn it into a batch of size 1
        if len(x.shape) == 2:
            x = torch.unsqueeze(x, 0)

        x = F.elu(self.cnv1(x))
        x = self.mp1(x)

        x = F.elu(self.cnv2(x))
        x = self.mp2(x)

        x = F.elu(self.cnv3(x))
        x = self.mp3(x)

        x = x.view(x.shape[0], np.prod(x.shape[1:]))

        #x = self.fc(x)

        x = F.elu(self.fc1(x))
        x = self.drp1(x)
        x = F.elu(self.fc2(x))
        x = self.drp2(x)
        x = self.fc3(x)
        x = F.softmax(input=x, dim=1)

        return x

    def load(self):
        if os.path.isfile(MODEL_FILENAME):
            self.load_state_dict(torch.load(MODEL_FILENAME))
            rospy.loginfo("Model parameters loaded.")

    def save(self):
        torch.save(self.state_dict(), MODEL_FILENAME)


# Training parameters

# Start by loading previously trained parameters.
# If this is False, training will start from scratch
USE_EXISTING_PARAMETERS = False

DISCOUNT_FACTOR = 0.99  # aka gamma

MAX_EPISODE_LENGTH = 500
# Sample neural net update batch from the replay memory.
# It contains this many steps.
MEMORY_SIZE = 10000

BATCH_SIZE = 128
LEARNING_RATE = 0.001

# Probability to select a random episode starts at EPS_START
# and reaches EPS_END once EPS_DECAY episodes are completed.
EPS_START = 1.0
EPS_END = 0.1
EPS_DECAY = 20000
