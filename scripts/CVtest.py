#!/usr/bin/env python3

from std_msgs.msg import Float64MultiArray, Bool, Float64
from robotic_arm_quark.msg import CVAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('robotic_arm_quark')


class CVtestServer:
    def __init__(self):

        self.server = actionlib.SimpleActionServer(
            'cv_actionServer', CVAction, self.execute, False)
        self.server.start()
        self.pose = [640, 360]

    def execute(self, goal):
        Controller()
        self.server.set_succeeded()


class Controller():
    def __init__(self):

        self.goal = [640/2, 360/2]
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
        while not rospy.is_shutdown:
            rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID)
            if self.reached():
                enable = False

                # terminate PIDs
                self.cam1.publish(enable)
                self.cam2.publish(enable)
                # exit from class
                break

    def reached(self):
        if abs((self.x - self.goal[0]) < 10) and abs((self.y - self.goal[1]) < 10):
            return True

    def sendToPID(self, coords):
        self.x = coords[0]
        self.y = coords[1]

        self.statePublisher1.publish(self.x)
        self.statePublisher2.publish(self.y)

        self.setpointPublisher1.publish(self.goal[0])
        self.setpointPublisher2.publish(self.goal[1])

if __name__ == '__main__':
    rospy.init_node('CV_test')
    server = CVtestServer()
    rospy.spin()
