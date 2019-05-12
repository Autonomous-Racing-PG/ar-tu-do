#!/usr/bin/env python

import rospy
from parameters import *
import car
import torch
import os

rospy.init_node('qdriver', anonymous=True)

timer = rospy.Rate(UPDATE_FREQUENCY)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
policy_net = NeuralQEstimator().to(device)
policy_net.load()

if not os.path.isfile(MODEL_FILENAME):
    message = "Model parameters for the neural net not found. You need to train it first."
    rospy.logerr(message)
    rospy.signal_shutdown(message)
    exit(1)

while not rospy.is_shutdown():
    scan = car.get_scan(LASER_SAMPLE_COUNT, device)
    with torch.no_grad():
        net_output = policy_net(scan)
        action_index = net_output.max(0)[1].item()
        action = ACTIONS[action_index]
        car.drive(*action)
    timer.sleep()
