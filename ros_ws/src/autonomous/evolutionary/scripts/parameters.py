import torch
import torch.nn as nn
import numpy as np

import rospy

from rospkg import RosPack
from drive_msgs.msg import drive_param


# TOPICS
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_SCAN = "/scan"
TOPIC_CRASH = "/crash"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"

STATE_SIZE = 8

MIN_SPEED = 0.2
MAX_SPEED = 0.4

POPULATION_SIZE = 10
SURVIVOR_COUNT = 4

LEARN_RATE = 1
normal_distribution = torch.distributions.normal.Normal(0, LEARN_RATE)

drive_parameters_publisher = rospy.Publisher(
    TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

class NeuralCarDriver(nn.Module):
    def __init__(self):
        super(NeuralCarDriver, self).__init__()

        self.layers = nn.Sequential(
            nn.Linear(STATE_SIZE, 8),
            nn.ReLU(),
            nn.Linear(8, 8),
            nn.ReLU(),
            nn.Linear(8, 2),
            nn.Tanh()
        )

        self.fitness = None

    def drive(self, state):
        with torch.no_grad():
            action = self.layers.forward(state)
        
        message = drive_param()
        message.angle = action[0].item()
        message.velocity = (MIN_SPEED + MAX_SPEED) / 2 + action[1].item() * (MAX_SPEED - MIN_SPEED) / 2
        drive_parameters_publisher.publish(message)

    def to_vector(self):
        state_dict = self.layers.state_dict()
        tensors = [state_dict[key] for key in sorted(state_dict.keys())]
        tensors = [torch.flatten(tensor) for tensor in tensors]
        vector = torch.cat(tensors)

        return vector

    def load_vector(self, vector):
        state_dict = self.layers.state_dict()
        
        position = 0
        for key in sorted(state_dict.keys()):
            old_value = state_dict[key]
            size = np.prod(old_value.shape)
            new_value = vector[position:position + size].reshape(old_value.shape)
            state_dict[key] = new_value
            position += size
        self.layers.load_state_dict(state_dict)

    def mutate(self):
        parameters = self.to_vector()
        parameters += normal_distribution.sample(parameters.shape)
        offspring = NeuralCarDriver()
        offspring.load_vector(parameters)
        return offspring