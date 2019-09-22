#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
import random
from std_msgs.msg import Empty
from gazebo_msgs.msg import ModelStates
import torch
from neural_car_driver import *
import simulation_tools.reset_car as reset_car

MAX_EPISODE_LENGTH = 5000
INITIAL_RANDOM_POPULATION_SIZE = 250
CONTINUE_TRAINING = False


class TrainingNode():
    def __init__(self):
        self.scan_indices = None
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)
        rospy.Subscriber(TOPIC_CRASH, Empty, self.on_crash)

        self.population = []

        self.is_terminal_step = False
        self.episode_length = 0
        self.generation = 0
        self.test = 0

        self.untested_population = []
        if CONTINUE_TRAINING:
            for i in range(POPULATION_SIZE):
                driver = NeuralCarDriver()
                driver.load(i)
                self.untested_population.append(driver)
        else:
            self.untested_population = [
                NeuralCarDriver() for _ in range(INITIAL_RANDOM_POPULATION_SIZE)]

        self.current_driver = self.untested_population[0]

    def on_receive_laser_scan(self, message):
        self.current_driver.drive(message)
        self.episode_length += 1

        if self.is_terminal_step or self.episode_length > MAX_EPISODE_LENGTH:
            self.on_complete_test()

    def get_fitness(self):
        return self.episode_length * \
            (self.current_driver.total_velocity / self.episode_length)

    def on_complete_test(self):
        self.current_driver.fitness = self.get_fitness()

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

        reset_car.reset_random(0, 0, self.generation % 2 == 0)

    def on_complete_generation(self):
        self.population.sort(key=lambda driver: driver.fitness, reverse=True)

        for i in range(POPULATION_SIZE):
            self.population[i].save(i)

        rospy.loginfo("Generation {:d}: Fitness of the population: {:s}".format(
            self.generation + 1,
            ", ".join(str(int(driver.fitness)) for driver in self.population)
        ))
        self.population = self.population[:SURVIVOR_COUNT]

        self.untested_population = list()
        for _ in range(POPULATION_SIZE - SURVIVOR_COUNT):
            parent = random.choice(self.population)
            offspring = parent.mutate()
            self.untested_population.append(offspring)

        self.current_driver = self.untested_population[0]
        self.generation += 1

    def on_crash(self, _):
        if self.episode_length > 5:
            self.is_terminal_step = True


rospy.init_node('evolutionary_training', anonymous=True)
reset_car.register_service()
node = TrainingNode()
rospy.spin()
