#!/usr/bin/env python3
from robotic_arm_quark.msg import CVAction, goHomeAction, SweepAction, setTopAction, goAheadAction, goDownAction, gripAction,goHomeActionFeedback,goAheadActionResult
import robotic_arm_quark.msg
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('my_pkg_name')
# refer server.txt for each server functionality
def feedback_cb(msg):
    print("feedback recieved : ",msg)


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
    client.send_goal(feedback_cb = feedback_cb)
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


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
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def goAhead_Server():
    client = actionlib.SimpleActionClient(
        'go_Ahead', robotic_arm_quark.msg.goAheadAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def goDown_server():
    client = actionlib.SimpleActionClient(
        'go_Down', robotic_arm_quark.msg.goDownAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def grip_server():
    client = actionlib.SimpleActionClient(
        'grip', robotic_arm_quark.msg.gripAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0

def visualServey():
    pass


def mastercontroller():
    while not rospy.is_shutdown():    
        rospy.init_node('goHome', anonymous=False)
        goHomeServer()
        print("successful goAhead")
        rospy.init_node('sweep', anonymous=False)
        sweep_server()
        print("successful sweep")
        rospy.init_node('setTop', anonymous=False)
        set_top_Server()
        print("successful setTopAction")
        rospy.init_node('goAhead', anonymous=False)
        goAhead_Server()
        print("successful goAheadAction")
        rospy.init_node('visualServey', anonymous=False)
        visualServey()
        rospy.init_node('goDown', anonymous=False)
        goDown_server()
        print("successful go_Down")
        rospy.init_node('gripServer', anonymous=False)
        grip_server()
        print("successful gripAction")
        goHomeServer()
        print("successful goAhead call2")
        goDown_server()
        print("successful go_Down call2")
        grip_server()
        print("successful gripAction call2")
        goHomeServer()
        print("successful goAhead call3")
        



if __name__ == '__main__':
    
    try:
        mastercontroller()
    except rospy.ROSInterruptException:
        print("error in the servers")
