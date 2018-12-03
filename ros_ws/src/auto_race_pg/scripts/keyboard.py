#!/usr/bin/env python

import rospy
from auto_race_pg.msg import drive_param

import sys, select, termios, tty

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key


speed = 0.5
turn = 0.25


if __name__=="__main__":

  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('keyboard', anonymous=True)

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = drive_param()


       msg.velocity = x*speed
       msg.angle = th*turn

       pub.publish(msg)

  except:
    print 'error'

  finally:
    msg = drive_param()


    msg.velocity = 0
    msg.angle = 0
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)