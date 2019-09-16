#!/usr/bin/env python

import torch
import torch.nn as nn
import numpy as np
import math
import os
import random

import rospy

from rospkg import RosPack
from drive_msgs.msg import drive_param
from sensor_msgs.msg import LaserScan

# TOPICS
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_SCAN = "/scan"
TOPIC_CRASH = "/crash"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"

STATE_SIZE = 8

MIN_SPEED = 0.1
MAX_SPEED = 0.4

POPULATION_SIZE = 10
SURVIVOR_COUNT = 4

RANDOM_ACTIONS = 0.05

LEARN_RATE = 0.1
normal_distribution = torch.distributions.normal.Normal(0, LEARN_RATE)

MODEL_FILENAME = os.path.join(
    RosPack().get_path("evolutionary"),
    "evolutionary_{:d}.to")

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
        self.scan_indices = None
        self.total_velocity = 0

    def drive(self, laser_message):
        if self.scan_indices is None:
            self.scan_indices = [int(i * (len(laser_message.ranges) - 1) / (STATE_SIZE - 1)) for i in range(STATE_SIZE)]  # nopep8

        values = [laser_message.ranges[i] for i in self.scan_indices]
        values = [v if not math.isinf(v) else 100 for v in values]
        state = torch.tensor(values, dtype=torch.float)

        with torch.no_grad():
            action = self.layers.forward(state)

        message = drive_param()
        if random.random() < RANDOM_ACTIONS:
            message.angle = random.uniform(-1, 1)
            message.velocity = random.uniform(0, 1)
        else:
            message.angle = action[0].item()
            message.velocity = (MIN_SPEED + MAX_SPEED) / 2 + \
                action[1].item() * (MAX_SPEED - MIN_SPEED) / 2

        self.total_velocity += message.velocity

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
            new_value = vector[position:position +
                               size].reshape(old_value.shape)
            state_dict[key] = new_value
            position += size
        self.layers.load_state_dict(state_dict)

    def mutate(self):
        parameters = self.to_vector()
        parameters += normal_distribution.sample(parameters.shape)
        offspring = NeuralCarDriver()
        offspring.load_vector(parameters)
        return offspring

    def load(self, index):
        self.load_state_dict(torch.load(MODEL_FILENAME.format(index)))
        rospy.loginfo("Model parameters loaded.")

    def save(self, index):
        torch.save(self.state_dict(), MODEL_FILENAME.format(index))


if __name__ == '__main__':
    rospy.init_node('evolutionary_driver', anonymous=True)
    driver = NeuralCarDriver()

    try:
        driver.load(0)
    except IOError:
        message = "Model parameters for the neural net not found. You need to train it first."
        rospy.logerr(message)
        rospy.signal_shutdown(message)
        exit(1)

    rospy.Subscriber(TOPIC_SCAN, LaserScan, driver.drive)
    rospy.spin()
