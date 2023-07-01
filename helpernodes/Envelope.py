#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64

class envelope():
    def __init__(self):
        rospy.init_node('Publish_control')
        rospy.loginfo("envelope initialized")
        pub = rospy.Publisher('/effortWrapper', Float64MultiArray, queue_size=1)
        rospy.Subscriber('/motor1/control_effort', Float64, self.callback1)
        rospy.Subscriber('/CV_pidx_sweep/control_effort', Float64, self.callback1)


        rospy.Subscriber('/motor2/control_effort', Float64, self.callback2)
        rospy.Subscriber('/motor3/control_effort', Float64, self.callback3)
        self.control_array = Float64MultiArray()
        self.control_array.data = [0, 0, 0]

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            pub.publish(self.control_array)
            rate.sleep()

    def callback1(self, data):
        self.control_array.data[0] = data

    def callback2(self, data):
        self.control_array[1] = data

    def callback3(self, data):
        self.control_array[2] = data


if __name__ == '__main__':
    envelope()

