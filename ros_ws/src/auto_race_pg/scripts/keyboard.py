#!/usr/bin/env python

import rospy
from race.msg import drive_param

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
















# pub = rospy.Publisher('sim_parameters', drive_sim, queue_size=10)




# import curses



# def vel_and_angle(data):
# 	speed = data.velocity
# 	turn = data.angle
# 	forward = 0
# 	left = 0

# 	stdscr = curses.initscr()
# 	curses.cbreak()
# 	stdscr.keypad(1)

# 	stdscr.refresh()

# 	key = ''
# 	while key != ord('q'):
# 	#	signal.setitimer(signal.ITIMER_REAL,0.05)
# 	#	key = input()
# 		key = stdscr.getch()
# 		stdscr.refresh()
# 	#	signal.alarm(0)
# 		if key == curses.KEY_UP: 
# 			forward = forward + 1
# 			stdscr.addstr(2, 20, "Up  ")
# 			stdscr.addstr(2, 25, '%.2f' % forward)
# 			stdscr.addstr(5, 20, "    ")
# 		elif key == curses.KEY_DOWN:
# 			forward = forward - 1
# 			stdscr.addstr(2, 20, "Down")
# 			stdscr.addstr(2, 25, '%.2f' % forward)
# 			stdscr.addstr(5, 20, "    ")
# 		if key == curses.KEY_LEFT:
# 			left = left + 1
# 			forward = forward + 1
# 			stdscr.addstr(3, 20, "left")
# 			stdscr.addstr(3, 25, '%.2f' % left)
# 			stdscr.addstr(5, 20, "    ")
# 		elif key == curses.KEY_RIGHT:
# 			left = left - 1
# 			forward = forward + 1
# 			stdscr.addstr(3, 20, "rgt ")
# 			stdscr.addstr(3, 25, '%.2f' % left)
# 			stdscr.addstr(5, 20, "    ")
# 		if key == curses.KEY_DC:
# 			left = 0
# 			forward = 0
# 			stdscr.addstr(5, 20, "Stop")
# 		msg = drive_sim()
# 		msg.velocity = forward * data.velocity
# 		msg.angle = left * data.angle
# 		pub.publish(msg)
# 	curses.endwin()


# def listener():
	# rospy.init_node('keyboard_talker', anonymous=True)
	# rospy.Subscriber('drive_parameters', drive_param, vel_and_angle)
	# rospy.spin()

