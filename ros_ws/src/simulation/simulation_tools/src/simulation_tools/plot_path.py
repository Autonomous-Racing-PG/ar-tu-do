#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Empty
import time
import numpy as np
from matplotlib import pyplot as plt
import os

from track_geometry import OUTER_WALLS, INNER_WALLS

DISTANCE_THRESHOLD = 1
MARGIN = 0.1

current_position = None
crashes = []
spawns = []
paths = []
current_path = []


def model_state_callback(message):
    global current_path, current_position
    current_position = np.array([
        message.pose[1].position.x,
        message.pose[1].position.y
    ])

    if len(current_path) > 0:
        distance = np.linalg.norm(current_position - current_path[-1]).item()
        if distance > DISTANCE_THRESHOLD:
            path = np.stack(current_path)
            paths.append(path)
            current_path = []

    if len(current_path) == 0:
        spawns.append(current_position)

    current_path.append(current_position)


def crash_callback(message):
    if current_position is not None:
        crashes.append(current_position)


rospy.init_node('plot_path', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/crash", Empty, crash_callback)

print("Recording path... Press CTRL+C to stop recording and create a plot.")

while not rospy.is_shutdown():
    time.sleep(0.1)

if len(current_path) > 0:
    paths.append(np.stack(current_path))

x_limits = (np.min(OUTER_WALLS[:, 0]) - MARGIN,
            np.max(OUTER_WALLS[:, 0]) + MARGIN)
y_limits = (np.min(OUTER_WALLS[:, 1]) - MARGIN,
            np.max(OUTER_WALLS[:, 1]) + MARGIN)
size = (x_limits[1] - x_limits[0], y_limits[1] - y_limits[0])

fig = plt.figure(figsize=(size[0] * 0.22, size[1] * 0.22))
plt.axis('off')
plt.xlim(*x_limits)
plt.ylim(*y_limits)

for path in paths:
    plt.plot(path[:, 0], path[:, 1], color='blue', linewidth=0.6)

if len(crashes) != 0:
    crashes = np.stack(crashes)
    plt.plot(crashes[:, 0], crashes[:, 1], 'rx', markersize=6)

if len(spawns) > 1:
    spawns = np.stack(spawns)
    plt.plot(spawns[:, 0], spawns[:, 1], 'go', markersize=2)

plt.plot(INNER_WALLS[:, 0], INNER_WALLS[:, 1], color='gray', linewidth=0.6)
plt.plot(OUTER_WALLS[:, 0], OUTER_WALLS[:, 1], color='gray', linewidth=0.6)

file_index = 0
filename = None
while filename is None or os.path.isfile(filename):
    filename = 'path-{:03d}.pdf'.format(file_index)
    file_index += 1

extent = plt.gca().get_window_extent().transformed(fig.dpi_scale_trans.inverted())
plt.savefig(filename, bbox_inches=extent)
print('\nPlot saved to {:s}'.format(os.path.abspath(filename)))
