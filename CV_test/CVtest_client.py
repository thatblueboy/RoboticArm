#!/usr/bin/env python3
import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import CVtestAction, CVtestGoal

roslib.load_manifest('robotic_arm_quark')


def call_server():
    if True:
        client = actionlib.SimpleActionClient(
            'cv_actionServer', CVtestAction)

        # Waits until the action server has started up and started
        client.wait_for_server()
        rospy.loginfo('server started')

        goal = CVtestGoal()

        client.send_goal(goal)


        client.wait_for_result()
        print("successful")
        return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        rospy.init_node('cv_node', anonymous=False)
        result=call_server()
        print(result)

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
