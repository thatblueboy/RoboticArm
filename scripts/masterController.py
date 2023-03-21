#!/usr/bin/env python3
import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64

import robotic_arm_quark.msg
from robotic_arm_quark.msg import (CVAction, SweepAction, goAheadAction,
                                   goAheadActionResult, goDownAction,
                                   goHomeAction, goHomeActionFeedback,
                                   gripAction, setTopAction)

roslib.load_manifest('my_pkg_name')
# refer server.txt for each server functionality


def feedback_cb(msg):
    print("feedback recieved : ", msg)


# def call_CVserver():
#     client = actionlib.SimpleActionClient(
#         'cv_actionServer', robotic_arm_quark.msg.CVAction)
#     client.wait_for_server()
#     client.wait_for_result()
    # result = client.getresult()   then return result
    # return 0


def goHomeServer():
    client = actionlib.SimpleActionClient(
        'go_home', robotic_arm_quark.msg.goHomeAction)
    client.wait_for_server()

    theta1 = Float64()
    theta2 = Float64()
    theta3 = Float64()
    theta4 = Float64()
    theta5 = Float64()
    theta6 = Float64()

    theta1.data, theta2.data, theta3.data, theta4.data, theta5.data, theta6.data = rospy.get_param('home_theta_1'), rospy.get_param(
        'home_theta_2'), rospy.get_param('home_theta_3'), rospy.get_param('home_theta_4'), rospy.get_param('home_theta_5'), rospy.get_param('home_theta_6')

    goal = robotic_arm_quark.msg.goHomeGoal()
    goal = [theta1, theta2, theta3, theta4, theta5, theta6]
    client.send_goal(goal)
    client.wait_for_result()
   
    return client.getresult()


def sweep_server():
    client = actionlib.SimpleActionClient(
        'sweep', robotic_arm_quark.msg.SweepAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def set_top_Server():
    client = actionlib.SimpleActionClient(
        'set_Top', robotic_arm_quark.msg.setTopAction)
    client.wait_for_server()

    
    theta1 = Float64()
    theta2 = Float64()
    theta3 = Float64()
    theta4 = Float64()
    theta5 = Float64()
    theta6 = Float64()

    theta1.data, theta2.data, theta3.data, theta4.data, theta5.data, theta6.data = rospy.get_param('top_theta_1'), rospy.get_param(
        'top_theta_2'), rospy.get_param('top_theta_3'), rospy.get_param('top_theta_4'), rospy.get_param('top_theta_5'), rospy.get_param('top_theta_6')
    
    goal = robotic_arm_quark.msg.goHomeGoal()
    goal = [theta1, theta2, theta3, theta4, theta5, theta6]
    client.send_goal(goal)

    client.wait_for_result()
    # result = client.getresult()   then return result
    return client.getresult()


def goAhead_Server():
    client = actionlib.SimpleActionClient(
        'go_Ahead', robotic_arm_quark.msg.goAheadAction)
    client.wait_for_server()
    reached = client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def goDown_server():
    client = actionlib.SimpleActionClient(
        'go_Down', robotic_arm_quark.msg.goDownAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def grip_server(goal):
    client = actionlib.SimpleActionClient(
        'grip', robotic_arm_quark.msg.gripAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def visualServey():
    pass


def mastercontroller():
    while not rospy.is_shutdown():
        result = goHomeServer()
        if result.Reached.data == True:
            print("Reached Home pose")

        sweep_server()
        print("successful sweep")

        set_top_Server()
        print("successful setTopAction")

        goAhead_Server()
        print("successful goAheadAction")

        visualServey()

        goDown_server()
        print("successful go_Down")

        goal = [0]
        grip_server(goal)
        print("successful gripAction")
        goHomeServer()
        print("successful goAhead call2")
        goDown_server()
        print("successful go_Down call2")
        goal = [1]
        grip_server(goal)
        print("successful gripAction call2")
        goHomeServer()
        print("successful goAhead call3")


if __name__ == '__main__':

    try:
        rospy.init_node('goHome', anonymous=False)
        mastercontroller()
    except rospy.ROSInterruptException:
        print("error in the servers")
