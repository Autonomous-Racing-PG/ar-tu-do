#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from std_msgs.msg import Duration
from math import floor

from collections import namedtuple

Point = namedtuple("Point", ["x", "y"])


def format_duration(duration):
    minutes = int(duration.to_sec() / 60)
    seconds = int(duration.to_sec()) % 60
    fraction = duration.to_sec() - floor(duration.to_sec())
    return str(minutes) + ":" + str(seconds).ljust(2, '0') + \
        "." + str(int(fraction * 100)).ljust(2, '0')


class Area():
    def __init__(self, center, extents):
        self.center = center
        self.extents = extents

    def contains(self, point):
        return abs(self.center.x - point.x) < self.extents.x \
            and abs(self.center.y - point.y) < self.extents.y


class Timer():
    def __init__(self, name, checkpoints):
        self.name = name
        self.checkpoints = checkpoints
        self.next_checkpoint = 0
        self.history = []
        self.start = None
        self.lap_time_publisher = rospy.Publisher(
            "/lap_time", Duration, queue_size=1)

    def update(self, position):
        if self.checkpoints[self.next_checkpoint].contains(position):
            self.next_checkpoint += 1
            if self.next_checkpoint == 2:
                rospy.loginfo("Lap started (" + self.name + ")")
                self.start = rospy.Time.now()
            if self.next_checkpoint >= len(self.checkpoints):
                self.complete_lap()
                self.next_checkpoint = 2
        elif self.next_checkpoint > 1 and self.next_checkpoint < len(self.checkpoints) - 1 and self.checkpoints[0].contains(position):
            self.next_checkpoint = 1
        elif self.next_checkpoint == 1 and any([area.contains(position) for area in self.checkpoints[2:-2]]):
            self.next_checkpoint = 0

    def complete_lap(self):
        time = rospy.Time.now()
        duration = time - self.start
        self.history.append(duration)
        if len(self.history) == 1:
            rospy.loginfo("Lap " + str(len(self.history)) + " (" + self.name + "): " +  # nopep8
                          format_duration(duration))
        else:
            average = rospy.Duration(sum([item.to_sec() for item in self.history]) / len(self.history))  # nopep8
            rospy.loginfo("Lap " + str(len(self.history)) + " (" + self.name + "): " +  # nopep8
                          format_duration(duration) + ", average: " + format_duration(average))  # nopep8
        self.start = time
        self.lap_time_publisher.publish(duration)


FINISH_LINE_1 = Area(Point(0, -0.5), Point(2.8, 1))
FINISH_LINE_2 = Area(Point(4, -0.5), Point(1.2, 1))
CHECKPOINT_1 = Area(Point(11, 7), Point(2, 2))
CHECKPOINT_2 = Area(Point(0, 4), Point(2, 2))
CHECKPOINT_3 = Area(Point(-14, 1), Point(2, 2))

forward_track = Timer("forward", (
    FINISH_LINE_1,
    FINISH_LINE_2,
    CHECKPOINT_1,
    CHECKPOINT_2,
    CHECKPOINT_3,
    FINISH_LINE_1,
    FINISH_LINE_2))

reverse_track = Timer("reverse", (
    FINISH_LINE_2,
    FINISH_LINE_1,
    CHECKPOINT_3,
    CHECKPOINT_2,
    CHECKPOINT_1,
    FINISH_LINE_2,
    FINISH_LINE_1))


def model_state_callback(message):
    if len(message.pose) < 2:
        return

    position = Point(message.pose[1].position.x, message.pose[1].position.y)
    forward_track.update(position)
    reverse_track.update(position)


rospy.init_node('lap_timer', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)

rospy.spin()
