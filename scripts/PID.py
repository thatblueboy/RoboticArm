#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64


class PID():
    def __init__(self):
        rospy.init_node('PID', anonymous=True)

        self.goalx = rospy.get_param("goalx")
        self.goalx = rospy.get_param("goaly")
        self.state = Float64()
        enable = Bool()
        enable = True

        rospy.Subscriber('/centroid_coordinates', Point, self.publish)
        self.enable = rospy.Publisher(
            '/sweep_pid/pid_enable', Bool, queue_size=10)
        self.enable.publish(enable)

        self.setpointPublisher = rospy.Publisher(
            '/sweep_pid/setpoint', Float64, queue_size=10)
        self.statePublisher = rospy.Publisher(
            '/sweep_pid/state', Float64, queue_size=10)
        rospy.spin()

    def publish(self, Coords):
        self.state = Coords.x
        self.setpointPublisher.publish(self.goalx)
        self.statePublisher.publish(self.state)


if __name__ == '__main__':
    PID()
