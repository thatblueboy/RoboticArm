#!/usr/bin/env python3
from robotic_arm_quark.msg import gripAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool , Int32
import roslib
roslib.load_manifest('robotic_arm_quark')


class grip_server:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'grip', gripAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        controller(goal)
        self.server.set_succeeded()
    

class controller:
    def __init__(self, goal):
        self.grip = rospy.Publisher(
            '/gripper_enable', Int32)
        self.grip.publish(goal.TopPose)
    # def __init__(self):
    #   self.
    #   pass


if __name__ == '__main__':
    rospy.init_node('grip_server')
    server = grip_server()
    rospy.spin()
