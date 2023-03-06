#!/usr/bin/env python3
import robotic_arm_quark.msg
from robotic_arm_quark.msg import CVAction
from robotic_arm_quark.msg import gripAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('')


def feedback_cb(msg):
    print("feedback recieved : ",msg)


def call_server():
    while True:
        client = actionlib.SimpleActionClient(
            'cv_actionServer', robotic_arm_quark.msg.CVAction)

        # Waits until the action server has started up and started
        client.wait_for_server()

        client.send_goal(feedback_cb = feedback_cb)
        client.wait_for_result()
        print("successful")



if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        rospy.init_node('cv_node', anonymous=False)
        call_server()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
