#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import gripAction



class grip_server:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('grip', gripAction, self.execute, False)
    self.server.start()

    self.pose = [0, 0, 0, 0, 0]

#   def execute(self, goal):
#     self.server.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = grip_server()
  rospy.spin()
