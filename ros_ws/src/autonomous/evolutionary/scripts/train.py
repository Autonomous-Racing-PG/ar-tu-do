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
import numpy as np
from collections import deque

MAX_EPISODE_LENGTH = 5000
INITIAL_RANDOM_POPULATION_SIZE = 100
CONTINUE_TRAINING = False

global_fitness_history = deque(maxlen=500)

class SpawnPoint():
    def __init__(self):
        self.position = random.random()
        self.angle = 0
        self.offset = 0
        self.forward = random.random() > 0.5

        self.steps_completed = 0
        self.fitness_history = deque(maxlen=100)

    def reset_car(self):
        reset_car.reset(self.position, self.angle, self.offset, self.forward)

    def get_difficulty(self, fitness):
        global_fitness_history.append(fitness)
        self.fitness_history.append(fitness)
        if len(self.fitness_history) < 10:
            return 1
        else:
            return np.mean(global_fitness_history) / np.mean(self.fitness_history)

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

        self.spawn_points = [SpawnPoint() for _ in range(20)]
        self.current_spawn_point = None

        self.start_episode()

    def on_receive_laser_scan(self, message):
        self.current_driver.drive(message)
        self.episode_length += 1

        if self.is_terminal_step or self.episode_length > MAX_EPISODE_LENGTH:
            self.on_complete_test()

    def get_fitness(self):
        raw_fitness = self.episode_length * \
            (self.current_driver.total_velocity / self.episode_length)
        return raw_fitness * self.current_spawn_point.get_difficulty(raw_fitness)

    def on_complete_test(self):
        self.current_driver.fitness_history.append(self.get_fitness())
        self.current_driver.fitness = np.mean(self.current_driver.fitness_history)
        self.population.append(self.current_driver)
        self.untested_population.remove(self.current_driver)
        self.current_spawn_point.steps_completed += self.episode_length ** 2
        self.current_driver = None

        if len(self.untested_population) == 0:
            self.on_complete_generation()
        else:
            self.start_episode()

        if len(self.untested_population) > POPULATION_SIZE and len(self.untested_population) % 10 == 0:
            print("Testing random drivers for the starting population, {:d} remaining...".format(len(self.untested_population)))
        self.test += 1

    def start_episode(self):
        self.current_driver = self.untested_population[0]
        self.episode_length = 0
        self.current_driver.total_velocity = 0
        self.is_terminal_step = False
        
        self.current_spawn_point = min(self.spawn_points, key=lambda spawn_point: spawn_point.steps_completed)
        self.current_spawn_point.reset_car()
        
        
    def on_complete_generation(self):
        self.population.sort(key=lambda driver: driver.fitness, reverse=True)

        for i in range(POPULATION_SIZE):
            self.population[i].save(i)

        rospy.loginfo("Generation {:d}: Fitness of the population: {:s}".format(
            self.generation + 1,
            ", ".join("{:0.0f} ({:d})".format(driver.fitness, len(driver.fitness_history)) for driver in self.population)
        ))

        survivors = self.population[:SURVIVOR_COUNT]
        self.untested_population = list(survivors)
        self.population = []
        for _ in range(POPULATION_SIZE - SURVIVOR_COUNT):
            parent = random.choice(survivors)
            offspring = parent.mutate()
            self.untested_population.append(offspring)

        self.generation += 1
        self.start_episode()

    def on_crash(self, _):
        if self.episode_length > 5:
            self.is_terminal_step = True


rospy.init_node('evolutionary_training', anonymous=True)
reset_car.register_service()
node = TrainingNode()
rospy.spin()
