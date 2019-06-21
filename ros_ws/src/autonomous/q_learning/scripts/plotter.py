#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
from parameters import *
import pyqtgraph as pg
import rospy
import sys
from q_learning.msg import EpisodeResult


def update_result_lists(msg):
    reward_list.append(msg.reward)
    length_list.append(msg.length)


def update_plot():
    reward_plot.setData(range(len(reward_list))[-50:], reward_list[-50:])
    length_plot.setData(range(len(length_list))[-50:], length_list[-50:])


reward_list = list()
length_list = list()

app = QtGui.QApplication(sys.argv)
app_window = pg.GraphicsWindow(title='Q-Learning Plotter')
main_plot = app_window.addPlot(title='Episodes')

main_plot.addLegend()

length_plot = main_plot.plot(
    pen='g', symbol='o', symbolPen='w', symbolBrush='g', symbolSize=8, name='length')
reward_plot = main_plot.plot(
    pen='r', symbol='o', symbolPen='w', symbolBrush='r', symbolSize=8, name='reward')

update_plot_timer = QtCore.QTimer()
update_plot_timer.timeout.connect(update_plot)
update_plot_timer.start(500)

rospy.init_node('q_learning_plotter', anonymous=True)
rospy.Subscriber(TOPIC_EPISODE_RESULT, EpisodeResult, update_result_lists)

if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    app.exec_()

rospy.spin()
