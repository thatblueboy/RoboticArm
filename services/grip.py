#!/usr/bin/env python3
from robotic_arm_quark.msg import gripAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('my_pkg_name')


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
            '/gripper_enable', Float64)
        self.enableMotor1.publish(goal.order)
    # def __init__(self):
    #   self.
    #   pass


if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = grip_server()
    rospy.spin()
