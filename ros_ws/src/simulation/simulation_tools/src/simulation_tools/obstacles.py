#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *

NUMBER_OF_OBSTACLES = 1

CUBE = '''
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
            </visual>
          </link>
    </model>
</sdf>
'''

class Obstacle():
    def __init__(self, id=0, width=0.27, length=0.55, height=0.3):
        self.width = width
        self.length = length
        self.height = height
        self.name = 'obstacle_{:2d}'.format(id)
        self.id = id

        self.delete()
        self.spawn()

    def delete(self):
        delete_model_service(self.name)

    def spawn(self):
        model_description = CUBE.format(self.length, self.width, self.height)
        orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
        pose = Pose(Point(0.0, -0.5, self.height / 2), orientation)
        spawn_model_service(self.name, model_description, "", pose, "world")

    def update(self):
        pass


rospy.init_node('obstacles', anonymous=True)
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
delete_model_service = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
spawn_model_service = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

obstacles = [Obstacle(id=i) for i in range(NUMBER_OF_OBSTACLES)]

def update_obstacles(_):
    for obstacle in obstacles:
        obstacle.update()

rospy.Timer(rospy.Duration(0.05, 0), update_obstacles)
rospy.spin()