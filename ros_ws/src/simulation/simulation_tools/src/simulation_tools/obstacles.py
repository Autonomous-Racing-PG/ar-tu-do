#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from simulation_tools.track import track
import random

NUMBER_OF_OBSTACLES = 10

OBSTACLE_SDF = '''
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.4">
    <model name="test_cube">
        <static>true</static>
        <link name='chassis'>
            <pose>0 0 0  0 0 0</pose>

            <collision name='collision'>
              <geometry>
                <box>
                  <size>{0:.2f} {1:.2f} {2:.2f}</size>
                </box>
              </geometry>
            </collision>

            <visual name='visual'>
              <geometry>
                <box>
                  <size>{0:.2f} {1:.2f} {2:.2f}</size>
                </box>
              </geometry>
              <material>
                 <diffuse>1 0 0 1</diffuse>
                 <ambient>1 0 0 1</ambient>
              </material>
            </visual>
          </link>
    </model>
</sdf>
'''

class Obstacle():
    def __init__(self, id=0, width=0.25, length=0.5, height=0.3):
        self.width = width
        self.length = length
        self.height = height
        self.name = 'obstacle_{:2d}'.format(id)
        self.id = id

        self.delete()
        self.spawn()

        self.lap_progress = random.uniform(0, 1)
        self.lap_time = 50

    def delete(self):
        delete_model_service(self.name)

    def spawn(self):
        model_description = OBSTACLE_SDF.format(self.length, self.width, self.height)
        orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose = Pose(Point(0.0, -0.5, self.height / 2), orientation)
        spawn_model_service(self.name, model_description, "", pose, "world")

    def update(self, delta_time):
        self.lap_progress += delta_time / self.lap_time
        if self.lap_progress > 1:
            self.lap_progress -= 1

        track_position = track.get_position(self.lap_progress * track.length)
        self.set_pose(track_position.point, -track_position.get_smooth_angle())        

    def set_pose(self, position, angle):
        state = ModelState()
        state.model_name = self.name
        state.pose.position.x = position.x
        state.pose.position.y = position.y
        state.pose.position.y = position.y

        q = quaternion_from_euler(angle, 0, 0)
        state.pose.orientation.x = q[0]
        state.pose.orientation.z = q[1]
        state.pose.orientation.w = q[2]
        state.pose.orientation.y = q[3]

        set_model_state_service(state)


rospy.init_node('obstacles', anonymous=True)
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
rospy.wait_for_service('/gazebo/set_model_state')
delete_model_service = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
spawn_model_service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

obstacles = [Obstacle(id=i) for i in range(NUMBER_OF_OBSTACLES)]
last_update = rospy.get_time()

while not rospy.is_shutdown():
    delta_time = rospy.get_time() - last_update
    last_update = rospy.get_time()
    for obstacle in obstacles:
        obstacle.update(delta_time)