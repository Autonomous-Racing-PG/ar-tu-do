#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from drive_msgs.msg import drive_param

flag_move = 0


def set_throttle_steer(data):
    """
    This method get's called when new drive_parameters are published.
    It then converts the single message into 6 new messages.
    The first 4 are the velocity of each individual wheels (velocity of the car).
    The last 2 are the steering hinge angle for each of the front wheels (steering angle of the car).

    Args:
        data:   A drive_param message
    """

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher(
        '/racer/left_wheel_back_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher(
        '/racer/right_wheel_back_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher(
        '/racer/left_wheel_front_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher(
        '/racer/right_wheel_front_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher(
        '/racer/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher(
        '/racer/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.velocity/0.1
    steer = data.angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)


def drive_param_converter():

    rospy.init_node('drive_param_converter', anonymous=True)

    rospy.Subscriber("/set/drive_param", drive_param, set_throttle_steer)

    rospy.spin()


if __name__ == '__main__':
    try:
        drive_param_converter()
    except rospy.ROSInterruptException:
        pass
