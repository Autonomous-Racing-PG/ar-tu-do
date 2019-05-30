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

        # We've got 2 input channels (laser scans and their derivatives)
        # and want 6 output channels
        #
        # if we want to keep the dimensions of the output the same
        # as the input (1080) we need to use a padding of 8
        # if we use a kernel size of 17
        self.cnv1 = nn.Conv1d(in_channels=2, out_channels=6, kernel_size=17, padding=8)
        # self.cnv2 = nn.Conv1d(6, 18, kernel_size=32)

        # same reasons as above
        self.mp1 = nn.MaxPool1d(kernel_size=9, padding=4)
        # self.cnv3 = nn.Conv1d(18, 24, kernel_size=16)
        # self.cnv4 = nn.Conv1d(24, 32, kernel_size=8)
        # self.mp2 = nn.MaxPool1d(4)

        self.fc1 = nn.Linear(720, 512)
        self.fc2 = nn.Linear(512, 128)
        self.fc3 = nn.Linear(128, ACTION_COUNT)

    def forward(self, x):
        # x = F.elu(self.fc1(x))
        # x = F.elu(self.fc2(x))
        # return self.fc3(x)

        # model = torch.nn.Sequential(
        #     self.cnv1,
        #     self.cnv2,
        #     self.mp1,
        #     self.cnv3,
        #     self.cnv4,
        #     self.mp2,
        #     self.fc1,
        #     nn.Dropout(p=0.6),
        #     nn.ELU(),
        #     self.fc2,
        #     nn.Dropout(p=0.6),
        #     nn.ELU(),
        #     self.fc3,
        #     nn.Softmax(dim=-1)
        # )
        # return model(x)

        # model = torch.nn.Sequential(
        #     self.cnv1,
        #     self.mp1,
        #     self.fc1,
        #     nn.Dropout(p=0.6),
        #     nn.ELU(),
        #     self.fc2,
        #     nn.Dropout(p=0.6),
        #     nn.ELU(),
        #     self.fc3,
        #     nn.Softmax(dim=-1)
        # )
        # return model(x)

        # Size changes from (128, 2, 1080) to (128, 6, 1080)
        x = self.cnv1(x)
        # print('shape nach cnv1:', x.shape)

        # Size changes from (128, 6, 1080) to (128, 6, 120)
        x = self.mp1(x)
        # print('shape nach mp1:', x.shape)
        # Size changes from (128, 6, 120) to (1, 92160)
        # where 92160 is the product of the shape after
        # the MaxPool Layer (128*6*120)
        # OR
        # (128, 720) (6*120)
        # print(x.shape[1:])
        x = x.view(128, np.prod(x.shape[1:]))
        # print('shape nach x.view', x.shape)
        x = self.fc1(x)
        # print('shape nach fc1', x.shape)
        x = F.elu(x)
        x = self.fc2(x)
        x = F.elu(x)
        x = self.fc3(x)

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
