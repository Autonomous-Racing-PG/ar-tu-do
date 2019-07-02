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
from simulation_tools.track import track, Point


class TrainingNode():
    def __init__(self):
        self.scan_indices = None
        self.drive_parameters_publisher = rospy.Publisher(
            TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)
        rospy.Subscriber(TOPIC_SCAN, LaserScan, self.on_receive_laser_scan)
        rospy.Subscriber(TOPIC_CRASH, Empty, self.on_crash)
        rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, self.on_model_state_callback)  # nopep8

        self.population = []
        self.untested_population = [NeuralCarDriver() for _ in range(POPULATION_SIZE)]
        self.current_driver = self.untested_population[0]

        self.is_terminal_step = False
        self.episode_length = 0
        self.generation = 0
        self.test = 0

        self.segment_start_time = None
        self.segment_times = []
        self.track_progress = None
    
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
        if len(self.segment_times) < 20:
            return self.track_progress
        return sum(200 / SEGMENT_COUNT - time for time in self.segment_times)

    def on_complete_test(self):
        self.current_driver.fitness = self.get_fitness()

        self.population.append(self.current_driver)
        self.untested_population.remove(self.current_driver)
        self.episode_length = 0
        self.is_terminal_step = False
        self.segment_start_time = rospy.Time.now()
        self.segment_times = []

        if len(self.untested_population) == 0:
            self.current_driver = None
            self.on_complete_generation()
        else:
            self.current_driver = self.untested_population[0]
        self.test += 1

        reset_car.reset()
    
    def on_complete_generation(self):
        self.population.sort(key=lambda driver: driver.fitness, reverse=True)
        rospy.loginfo("Generation {}: Fitness of the population: ".format(self.generation + 1) + ", ".join("{:.1f}".format(driver.fitness) for driver in self.population))
        self.population = self.population[:SURVIVOR_COUNT]

        self.untested_population = list()
        for _ in range(POPULATION_SIZE - SURVIVOR_COUNT):
            if USE_CROSSOVER:
                # do uniform crossover randomly between best individuals
                parent = random.choice(self.population[:SURVIVOR_COUNT])
                parent2 = random.choice(self.population[:SURVIVOR_COUNT])
                offspring = parent.crossover_uniform(parent2)

                # mutate offspring with random probability
                if random.random() > 0.5:
                    offspring = offspring.mutate()
            else:
                offspring = random.choice(self.population).mutate()

            self.untested_population.append(offspring)
        
        self.current_driver = self.untested_population[0]
        self.generation += 1

    def on_crash(self, _):
        if self.episode_length > 5:
            self.is_terminal_step = True

    def on_model_state_callback(self, message):
        if len(message.pose) < 2:
            return
        car_position = Point(
            message.pose[1].position.x,
            message.pose[1].position.y)

        track_position = track.localize(car_position)
        self.track_progress = track_position.total_progress
        current_segment = int(self.track_progress * SEGMENT_COUNT)
        if current_segment != 1 and len(self.segment_times) == 0:
            return
        if current_segment > len(self.segment_times) or (current_segment == 0 and len(self.segment_times) > 2):
            time = (rospy.Time.now() - self.segment_start_time).to_sec()
            if time > 0:
                self.segment_times.append(time)
            self.segment_start_time = rospy.Time.now()
            if len(self.segment_times) == SEGMENT_COUNT + 1:
                self.is_terminal_step = True
                rospy.loginfo("Lap time: {:.0f}".format(sum(self.segment_times)))


rospy.init_node('evolutionary_training', anonymous=True)
reset_car.register_service()
node = TrainingNode()
rospy.spin()
