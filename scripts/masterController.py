#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import gripAction
from robotic_arm_quark.msg import CVAction
import robotic_arm_quark.msg


def main():
    rospy.init_node('master_node', anonymous = False)
    

    # call CV action
    rospy.init_node('master_client')
    client = actionlib.SimpleActionClient('master', robotic_arm_quark.msg.CVAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Sends the goal to the action server.
   

    # goal = [640/2 ,360/2]
    # # Creates a goal to send to the action server.
    # client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return 0

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        main()
        print("successful")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)