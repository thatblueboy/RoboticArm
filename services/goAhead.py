#!/usr/bin/env python3

from math import acos, asin, atan, cos, sin, tan

import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import goAheadAction
import math

roslib.load_manifest('robotic_arm_quark')

# take input from CV on /feedback, in callback, publish a constant value(r(radial velocity, as float64)) to a topic
# once a contour is found in feed and it has travelled sufficient distance into the frame, publish velocity as 0, service successful


class goAhead_Server:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'go_Ahead', goAheadAction, self.execute, False)
        self.server.start()

        rospy.spin()

    def execute(self, goal):
        rospy.loginfo('in excute')
        l1 = rospy.get_param('/link1')
        l2 = rospy.get_param('/link2')
        goAhead(l1, l2)

        self.server.set_succeeded()


class goAhead():

    def __init__(self, l1, l2):
        rospy.loginfo('in goAhead class')
        self.theta = [0,0]
        self.enable = Bool()
        self.enable = False
        self.reached = False
        # self.theta1 = 0
        # self.theta2 = 0
        self.l1 = l1
        self.l2 = l2
        self.t = 0.5
        self.theta1 = 0
        self.theta2 = 0
        self.set = 0
        enable = Bool()
        enable = True
        self.x = rospy.get_param('x')
        self.y = rospy.get_param('z')  # z is height in param

        self.sub2 = rospy.Subscriber('/feedback', Float64, self.callback1)

        # self.SubState2 = rospy.Subscriber(
        #     '/motor2/state', Float64, self.getState2)
        # self.SubState3 = rospy.Subscriber(
        #     '/motor3/state', Float64, self.getState3)

        self.enableMotor2 = rospy.Publisher(
            '/motor2/pid_enable', Bool, queue_size=10)
        self.enableMotor3 = rospy.Publisher(
            '/motor3/pid_enable', Bool, queue_size=10)

        self.enableMotor2.publish(enable)
        self.enableMotor3.publish(enable)

        self.setpointPublisherMotor2 = rospy.Publisher(
            '/motor2/setpoint', Float64, queue_size=10)
        self.setpointPublisherMotor3 = rospy.Publisher(
            '/motor3/setpoint', Float64, queue_size=10)

        rospy.loginfo('about to enter while loop')
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('in loop')
            if self.reached:
                #get thetas from /motor2/state and /motor3/state by subscribing ONCE(get one message)
                # calculate x from this data and set the param to this value
                self.fk()

                rospy.set_param('/x', x)
                break

        rospy.loginfo('loop ended')
    # def getState2(self, theta2):
    #     self.theta2 = theta2

    # def getState3(self, theta3):
    #     self.theta3 = theta3
    def fk(self,l1,l2):
        rospy.Subscriber("/motor2/state", Float64, self.callback_theta1)
        rospy.Subscriber("/motor3/state", Float64, self.callback_theta2)
        self.x = (l1 + (l2*math.cos(self.theta[1]))*math.cos(self.theta[0])) 
        self.y = (l1 + (l2*math.cos(self.theta[1]))*math.sin(self.theta[0]))
    def callback_theta1(self,data):
        self.theta[0] = data.data
    def callback_theta2(self,data):
        self.theta[1] = data.data
    def callback1(self, data):
        coords = data.data
        self.set = coords[1]
        self.calculate()

    def calculate(self):
        # IK
        # if out of frame
        print('x %i' % self.set)
        if self.set == -1:
            theta2 = acos(((self.x*self.x)+(self.y*self.y) -
                          (self.l2*self.l2)-(self.l1*self.l1))/(2*self.l1*self.l2))
            theta1 = atan(self.y/self.x)-atan(self.l2 *
                                              sin(self.theta2)/(self.l1+self.l2*cos(self.theta2)))
            self.goTo(theta1, theta2)
            self.x = + 0.5
        else:
            self.reached = True

    def goTo(self, theta1, theta2):
        while (self.AngleNotReached(theta1, theta2)):
            self.setpointPublisherMotor2.publish(self.theta1)
            self.setpointPublisherMotor3.publish(self.theta2)
        # publish setpoint as theta1 and theta2

    def AngleNotReached(self, theta1, theta2):
        if abs(self.theta1 - theta1) < 0.01 and abs(self.theta2 - theta2) < 0.01:
            return True
        return False


if __name__ == '__main__':
    rospy.init_node('goahead_server')
    rate = rospy.Rate(1)

    server = goAhead_Server()
    rospy.spin()
