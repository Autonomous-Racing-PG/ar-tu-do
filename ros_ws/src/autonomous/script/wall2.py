#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param

import numpy as np

import matplotlib.pyplot as plt


def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def laser_callback(message):
    global laser_scan
    laser_scan = message


def handle_scan():
    if laser_scan is None:
        return
    ranges = np.array(laser_scan.ranges)
    sample_count = ranges.shape[0]
    angles = np.arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)

    points = np.zeros((ranges.shape[0], 2))
    points[:, 0] = -np.sin(angles) * ranges
    points[:, 1] = np.cos(angles) * ranges

    relative = points[1:, :] - points[:-1,:]
    distances = np.linalg.norm(relative, axis=1)

    margin_relative = 0.1
    margin = int(sample_count * margin_relative)
    split = margin + np.argmax(distances[margin:-margin]) + 1
    
    plt.clf()
    plt.xlim(-7, 7)
    plt.ylim(-2, 12)
    plt.gca().set_aspect("equal")
    plt.scatter(points[:split,0], points[:split,1], marker="x", color="red")
    plt.scatter(points[split:,0], points[split:,1], marker="x", color="blue")
    plt.draw()
    plt.pause(0.001)
        


laser_scan = None

rospy.Subscriber("/scan", LaserScan, laser_callback)
drive_parameters_publisher = rospy.Publisher(
    "/input/drive_param/wallfollowing", drive_param, queue_size=1)


rospy.init_node('wall2', anonymous=True)

timer = rospy.Rate(20)
plt.ion()
    

while not rospy.is_shutdown():
    handle_scan()
    timer.sleep()