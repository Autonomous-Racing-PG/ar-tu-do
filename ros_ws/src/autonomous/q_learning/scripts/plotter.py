#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
from parameters import *
import pyqtgraph as pg
import rospy
import sys
import numpy
from collections import deque
from std_msgs.msg import Float32


def update_reward_list(msg):
    reward_list.append(msg.data)


def update_plot():
    reward_plot.plot(reward_list, clear=True)


reward_list = deque(maxlen=50)

app = QtGui.QApplication(sys.argv)

reward_plot = pg.plot()

update_plot_timer = QtCore.QTimer()
update_plot_timer.timeout.connect(update_plot)
update_plot_timer.start(500)

rospy.init_node('q_learning_plotter', anonymous=True)
rospy.Subscriber(TOPIC_REWARDS, Float32, update_reward_list)

if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    app.exec_()

rospy.spin()
