#!/usr/bin/env python

from PyQt4 import QtGui
from parameters import *
import pyqtgraph as pg
import rospy
import sys
from collections import deque
from q_learning.msg import EpisodeResult


def update_plot(msg):
    global episode_count, plot_window
    reward_list.append(msg.reward)
    length_list.append(msg.length)
    episode_count += 1
    reward_plot.prepareGeometryChange()
    reward_plot.setData(range(episode_count)[-plot_window:], reward_list)
    length_plot.prepareGeometryChange()
    length_plot.setData(range(episode_count)[-plot_window:], length_list)


rospy.init_node('q_learning_plotter', anonymous=True)

plot_window = rospy.get_param("~plot_window")

reward_list = deque(maxlen=plot_window)
length_list = deque(maxlen=plot_window)
episode_count = 0

app = QtGui.QApplication(sys.argv)
app_window = pg.GraphicsWindow(title='Q-Learning Plotter')
main_plot = app_window.addPlot(title='Episodes')

main_plot.addLegend()

length_plot = main_plot.plot(
    pen='g',
    name='length')
reward_plot = main_plot.plot(
    pen='r',
    name='reward')

rospy.Subscriber(TOPIC_EPISODE_RESULT, EpisodeResult, update_plot)

if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    app.exec_()

rospy.spin()
