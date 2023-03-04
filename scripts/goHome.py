#!/usr/bin/env python3

from robotic_arm_quark.msg import goHomeAction  # what is this importing??
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, Bool, Float64
import roslib
roslib.load_manifest('robotic_arm_quark')


class goHomeServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'go_home', goHomeAction, self.execute, False)
        self.server.start()
        self.goalPose = [0, 0, 0, 0, 0]  # change goal pose from here

    def execute(self):
        Controller(self.goalPose)
        self.server.set_succeeded()


class Controller:
    def __init__(self, array):
        enable = Bool()
        enable = True

        self.stateArray = [0, 0, 0, 0, 0]

        self.theta1 = Float64()
        self.theta2 = Float64()
        self.theta3 = Float64()
        self.theta4 = Float64()
        self.theta5 = Float64()

        self.theta1 = array[0]
        self.theta2 = array[1]
        self.theta3 = array[2]
        self.theta4 = array[3]
        self.theta5 = array[4]

        self.goalArray = [self.theta1,
                          self.theta2,
                          self.theta3,
                          self.theta4,
                          self.theta5]

        self.enableMotor1 = rospy.Publisher(
            '/motor1/pid_enable', Bool, queue_size=10)
        self.enableMotor2 = rospy.Publisher(
            '/motor2/pid_enable', Bool, queue_size=10)
        self.enableMotor3 = rospy.Publisher(
            '/motor3/pid_enable', Bool, queue_size=10)
        self.enableMotor4 = rospy.Publisher(
            '/motor4/pid_enable', Bool, queue_size=10)
        self.enableMotor5 = rospy.Publisher(
            '/motor5/pid_enable', Bool, queue_size=10)

        # start PIDs
        self.enableMotor1.publish(enable)
        self.enableMotor2.publish(enable)
        self.enableMotor3.publish(enable)
        self.enableMotor4.publish(enable)
        self.enableMotor5.publish(enable)

        self.setpointPublisherMotor1 = rospy.Publisher(
            '/motor1/setpoint', Float64, queue_size=10)
        self.setpointPublisherMotor2 = rospy.Publisher(
            '/motor2/setpoint', Float64, queue_size=10)
        self.setpointPublisherMotor3 = rospy.Publisher(
            '/motor3/setpoint', Float64, queue_size=10)
        self.setpointPublisherMotor4 = rospy.Publisher(
            '/motor4/setpoint', Float64, queue_size=10)
        self.setpointPublisherMotor5 = rospy.Publisher(
            '/motor5/setpoint', Float64, queue_size=10)

        self.statePublisherMotor1 = rospy.Publisher(
            '/motor1/state', Float64, queue_size=10)
        self.statePublisherMotor2 = rospy.Publisher(
            '/motor2/state', Float64, queue_size=10)
        self.statePublisherMotor3 = rospy.Publisher(
            '/motor3/state', Float64, queue_size=10)
        self.statePublisherMotor4 = rospy.Publisher(
            '/motor4/state', Float64, queue_size=10)
        self.statePublisherMotor5 = rospy.Publisher(
            '/motor5/state', Float64, queue_size=10)

        while not rospy.is_shutdown:
            rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID)
            if self.reached():
                enable = False

                # terminate PIDs
                self.enableMotor1.publish(enable)
                self.enableMotor2.publish(enable)
                self.enableMotor3.publish(enable)
                self.enableMotor4.publish(enable)
                self.enableMotor5.publish(enable)

                # exit from class
                break

    def sendToPID(self, array):
        # publish state(curent angles)
        self.state = array
        self.statePublisherMotor1.publish(array[0])
        self.statePublisherMotor2.publish(array[1])
        self.statePublisherMotor3.publish(array[2])
        self.statePublisherMotor4.publish(array[3])
        self.statePublisherMotor5.publish(array[4])

        # publish goal
        self.setpointPublisherMotor1.publish(self.goalArray[0])
        self.setpointPublisherMotor2.publish(self.goalArray[0])
        self.setpointPublisherMotor3.publish(self.goalArray[0])
        self.setpointPublisherMotor4.publish(self.goalArray[0])
        self.setpointPublisherMotor5.publish(self.goalArray[0])

    def reached(self):
        if abs(self.stateArray[0] - self.goalArray[0]) < 0.4 and abs(self.stateArray[1] - self.goalArray[1] < 0.4) and abs(self.stateArray[2] - self.goalArray[2])<0.4 and abs(self.stateArray[3] - self.goalArray[3])<0.4 and abs(self.stateArray[0] - self.goalArray[0]):
            return True


if __name__ == '__main__':
    rospy.init_node('goHome_server')
    server = goHomeServer()
    rospy.spin()
