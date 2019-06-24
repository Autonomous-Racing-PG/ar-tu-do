#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
import random
from std_msgs.msg import Empty
from gazebo_msgs.msg import ModelStates
import torch
from parameters import *
import simulation_tools.reset_car as reset_car


class TrainingNode():
    def __init__(self):
        self.scan_indices = None
        self.drive_parameters_publisher = rospy.Publisher(
            TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)
        rospy.Subscriber(TOPIC_CRASH, Empty, self.on_crash)

        self.population = []
        self.untested_population = [NeuralCarDriver() for _ in range(POPULATION_SIZE)]
        self.current_driver = self.untested_population[0]

        self.is_terminal_step = False
        self.episode_length = 0
        self.generation = 0
        self.test = 0
    
    def convert_laser_message_to_tensor(self, message):
        if self.scan_indices is None:
            global scan_indices
            count = (message.angle_max - message.angle_min) / \
                message.angle_increment
            scan_indices = [int(i * count / STATE_SIZE) for i in range(STATE_SIZE)]

        values = [message.ranges[i] for i in scan_indices]
        values = [v if not math.isinf(v) else 100 for v in values]
        return torch.tensor(values, dtype=torch.float)

    def on_receive_laser_scan(self, message):
        state = self.convert_laser_message_to_tensor(message)
        self.current_driver.drive(state)
        self.episode_length += 1

        if self.is_terminal_step:
            self.on_complete_test()

    def get_fitness(self):
        return self.episode_length

    def on_complete_test(self):
        self.current_driver.fitness = self.get_fitness()

        rospy.loginfo("Generation: {0}, Test: {1}, Fitness: {2}".format(self.generation, self.test, self.current_driver.fitness))
        self.population.append(self.current_driver)
        self.untested_population.remove(self.current_driver)
        self.episode_length = 0
        self.is_terminal_step = False

        if len(self.untested_population) == 0:
            self.current_driver = None
            self.on_complete_generation()
        else:
            self.current_driver = self.untested_population[0]
        self.test += 1

        reset_car.reset()
    
    def on_complete_generation(self):
        self.population.sort(key=lambda driver: driver.fitness, reverse=True)
        self.population = self.population[:SURVIVOR_COUNT]

        self.untested_population = list(self.population)
        for _ in range(POPULATION_SIZE - SURVIVOR_COUNT):
            parent = random.choice(self.population)
            offspring = parent.mutate()
            self.untested_population.append(offspring)
        
        self.population = []
        self.current_driver = self.untested_population[0]
        self.generation += 1

    def on_crash(self, _):
        if self.episode_length > 5:
            self.is_terminal_step = True


rospy.init_node('evolutionary_training', anonymous=True)
reset_car.register_service()
node = TrainingNode()
rospy.spin()