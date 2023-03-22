#!/usr/bin/env python3

import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import SweepGoal, SweepAction, SweepFeedback, SweepResult

roslib.load_manifest('robotic_arm_quark')


class CVtestServer:
    def __init__(self):
        rospy.loginfo("server initialized")
        # code added
        self.feedback = SweepFeedback()
        self.result = SweepResult()

        self.server = actionlib.SimpleActionServer(
            '/sweep', SweepAction, self.execute, False)

        self.server.start()
        # x = rospy.get_param("camfeed_centre_x")
        # y = rospy.get_param("camfeed_centre_y")

        # self.goal = [x, y]

    def execute(self, goal):
        rospy.loginfo("in callback")
        rospy.loginfo(goal)
        rospy.loginfo("goal printed")
        x = goal.Xcentre.data
        print("goal x is %i" % x)

      #   y = goal.centreOfFrame[1].data

        # print('execute')
        Controller(x)

        # # need to add a result and publish it back
        rospy.loginfo('%s: Succeeded' % "sweep server")
        self.result.recognised.data = True
        self.server.set_succeeded(self.result)


class Controller():

    def __init__(self, goal):
        rospy.loginfo('in controller')
        self.x = 0

        self.goal = goal
        rospy.loginfo(self.goal)
        enable = Bool()
        enable = True
        # self.override = rospy.Publisher()  need to override pid if object is not in range
        self.cam1 = rospy.Publisher(
            'CV_pidx/pid_enable', Bool, queue_size=10)

        self.cam1.publish(enable)

        self.setpointPublisher1 = rospy.Publisher(
            'CV_pidx/setpoint', Float64, queue_size=10)

        self.statePublisher1 = rospy.Publisher(
            'CV_pidx/state', Float64, queue_size=10)
        self.statePublisher2 = rospy.Publisher(
            'CV_pidy/state', Float64, queue_size=10)

        rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID)

        while not rospy.is_shutdown():
            rate.sleep()  # Sleeps for 1/rate sec

            if self.reached():
                rospy.loginfo('reached')
                enable = False
                # terminate PIDs
                self.cam1.publish(enable)
            #     self.cam2.publish(enable)
                break

        # rospy.spin()

    def reached(self):
        rospy.loginfo('x is %i' % (self.x))
        rospy.loginfo('xgoal is %i' % (self.goal))

        rospy.loginfo('x diff is %i' % (self.x - self.goal))
        # rospy.loginfo(self.goal)

        if abs((self.x - self.goal)) < 10:

            return True
        else:
            return False

    def null(self, coords):
        print('hello')

    def sendToPID(self, coords):
        # print(coords.data[0], coords.data[1])

        # rate.sleep() # Sleeps for 1/rate sec

        # rospy.loginfo('sending coordinates to pid')
        self.x = coords.data[0]
        if self.x == -1:
            pass

        else:
            # print('self x and y are', self.x, self.y)
            self.statePublisher1.publish(self.x)

            self.setpointPublisher1.publish(self.goal)


if __name__ == '__main__':
    rospy.init_node('CV_test')
    rate = rospy.Rate(1)

    server = CVtestServer()
