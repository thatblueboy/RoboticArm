from math import acos, asin, atan, cos, sin, tan

import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import goAheadAction

roslib.load_manifest('robotic_arm_quark')

# take input from CV on /feedback, in callback, publish a constant value(r(radial velocity, as float64)) to a topic
# once a contour is found in feed and it has travelled sufficient distance into the frame, publish velocity as 0, service successful


class goAhead_Server:
    def __init__(self, cmd_vel):
        self.server = actionlib.SimpleActionServer(
            'go_Ahead', goAheadAction, self.execute, False)
        self.server.start()

        self.pose = [0, 0, 0, 0, 0]

        rospy.spin()

    def execute(self, goal):
        l1 = 1
        l2 = 2
        goAhead(l1, l2)

        self.server.set_succeeded()


class goAhead():
    def __init__(self, l1, l2):
        self.enable = Bool()
        self.enable = False
        self.reached = False
        self.theta1 = 0
        self.theta2 = 0
        self.l1 = l1
        self.l2 = l2
        self.t = 0.5
        self.theta1 = 0
        self.theta2 = 0
        self.sub2 = rospy.Subscriber('/feedback', Float64, self.callback1)

        self.SubState2 = rospy.Subscriber(
            '/motor2/state', Float64, self.getState2)
        self.SubState3 = rospy.Subscriber(
            '/motor3/state', Float64, self.getState3)

        self.set = 0

        self.enableMotor2 = rospy.Publisher(
            '/motor2/pid_enable', Bool, queue_size=10)
        self.enableMotor3 = rospy.Publisher(
            '/motor3/pid_enable', Bool, queue_size=10)

        enable = Bool()
        enable = True

        self.enableMotor2.publish(enable)
        self.enableMotor3.publish(enable)

        self.setpointPublisherMotor2 = rospy.Publisher(
            '/motor2/setpoint', Float64, queue_size=10)
        self.setpointPublisherMotor3 = rospy.Publisher(
            '/motor3/setpoint', Float64, queue_size=10)

        # self.calculate()

        # FK
        # self.x=cos(self.theta1)+cos(self.theta2)
        # self.y=sin(self.theta1)+sin(self.theta2)

        # self.setpointPublisherMotor1 = rospy.Publisher(
        #     '/motor1/setpoint', Float64, queue_size=10)
        # self.setpointPublisherMotor2 = rospy.Publisher(
        #     '/motor2/setpoint', Float64, queue_size=10)

        # self.statePublisherMotor1 = rospy.Publisher(
        #     '/motor1/state', Float64, queue_size=10)
        # self.statePublisherMotor2 = rospy.Publisher(
        #     '/motor2/state', Float64, queue_size=10)

        while not rospy.is_shutdown:
            if self.reached:
                # exit from class
                break

    def getState2(self, theta2):
        self.theta2 = theta2

    def getState3(self, theta3):
        self.theta3 = theta3

    def callback1(self, data):
        coords = data.data
        self.set = coords[0]
        self.calculate()

    def calculate(self):
        # IK
        # if out of frame
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
    rospy.init_node('do_dishes_server')
    server = goAhead_Server()
    rospy.spin()
