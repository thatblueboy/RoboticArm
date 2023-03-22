#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64


def main():
    rospy.init_node('Publish_control')
    pub = rospy.Publisher('control', Float64)
    rospy.Subscriber('/motor1/control_effort', Float64, callback1)
    rospy.Subscriber('/motor2/control_effort', Float64, callback2)
    rospy.Subscriber('/motor3/control_effort', Float64, callback3)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        pub.publish(control_array)
        rate.sleep()

def callback1(data):
    control_array[0] = data

def callback2(data):
    control_array[1] = data

def callback3(data):
    control_array[2] = data


if __name__ == '__main__':
    control_array = [0, 0, 0]

    main()