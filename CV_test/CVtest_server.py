#!/usr/bin/env python3

import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import CVtestAction

roslib.load_manifest('robotic_arm_quark')


class CVtestServer:
    def __init__(self):
        rospy.loginfo("hello")

        self.server = actionlib.SimpleActionServer(
            'cv_actionServer', CVtestAction, self.execute, False)

        self.server.start()
        x = rospy.get_param("camfeed_centre_x")
        y = rospy.get_param("camfeed_centre_y")

        self.goal = [x, y]

    def execute(self, goal):
        rospy.loginfo("in callback")

        # print('execute')
        Controller(self.goal)

        # need to add a result and publish it back
        self.server.set_succeeded()


class Controller():

    def __init__(self, goal):
        rospy.loginfo('in controller')
        self.x = 0
        self.y = 0

        self.goal = goal
        enable = Bool()
        enable = True

        self.cam1 = rospy.Publisher(
            'CV_pidx/pid_enable', Bool, queue_size=10)
        self.cam2 = rospy.Publisher(
            'CV_pidy/pid_enable', Bool, queue_size=10)

        self.cam1.publish(enable)
        self.cam2.publish(enable)

        self.setpointPublisher1 = rospy.Publisher(
            'CV_pidx/setpoint', Float64, queue_size=10)
        self.setpointPublisher2 = rospy.Publisher(
            'CV_pidy/setpoint', Float64, queue_size=10)

        self.statePublisher1 = rospy.Publisher(
            'CV_pidx/state', Float64, queue_size=10)
        self.statePublisher2 = rospy.Publisher(
            'CV_pidy/state', Float64, queue_size=10)

        while not rospy.is_shutdown():
            rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID)

            if self.reached():
                rospy.loginfo('reached')
                enable = False
                # terminate PIDs
                self.cam1.publish(enable)
                self.cam2.publish(enable)
                break

        # rospy.spin()

    def reached(self):
        rospy.loginfo('x diff is %i' % (self.x - self.goal[0]))

        if abs((self.x - self.goal[0])) < 10 and abs((self.y - self.goal[1])) < 10:
            return True
        else:
            return False

    def null(self, coords):
        print('hello')

    def sendToPID(self, coords):

        rospy.loginfo('sending coordinates to pid')
        self.x = coords.data[0]
        self.y = coords.data[1]
        self.statePublisher1.publish(self.x)
        self.statePublisher2.publish(self.y)
        self.setpointPublisher1.publish(self.goal[0])
        self.setpointPublisher2.publish(self.goal[1])


if __name__ == '__main__':
    rospy.init_node('CV_test')
    server = CVtestServer()
