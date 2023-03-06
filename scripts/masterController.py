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


def call_CVserver():
    client = actionlib.SimpleActionClient(
        'cv_actionServer', robotic_arm_quark.msg.CVAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def call_server1():
    client = actionlib.SimpleActionClient(
        'go_home', robotic_arm_quark.msg.goHomeAction)
    client.wait_for_server()
    client.send_goal(feedback_cb = feedback_cb)
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def call_server2():
    client = actionlib.SimpleActionClient(
        'sweep', robotic_arm_quark.msg.SweepAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def call_server3():
    client = actionlib.SimpleActionClient(
        'set_Top', robotic_arm_quark.msg.setTopAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def call_server4():
    client = actionlib.SimpleActionClient(
        'go_Ahead', robotic_arm_quark.msg.goAheadAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def call_server5():
    client = actionlib.SimpleActionClient(
        'go_Down', robotic_arm_quark.msg.goDownAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def call_server6():
    client = actionlib.SimpleActionClient(
        'grip', robotic_arm_quark.msg.gripAction)
    client.wait_for_server()
    client.wait_for_result()
    # result = client.getresult()   then return result
    return 0


def mastercontroller():
    while not rospy.is_shutdown():    
        rospy.init_node('master_node1', anonymous=False)
        call_server1()
        print("successful goAhead")
        rospy.init_node('master_node2', anonymous=False)
        call_server2()
        print("successful sweep")
        rospy.init_node('master_node3', anonymous=False)
        call_server3()
        print("successful setTopAction")
        rospy.init_node('master_node4', anonymous=False)
        call_server4()
        print("successful goAheadAction")
        rospy.init_node('master_node5', anonymous=False)
        call_server5()
        print("successful go_Down")
        rospy.init_node('cv_node', anonymous=False)
        call_server6()
        print("successful gripAction")
        call_server1()
        print("successful goAhead call2")
        call_server5()
        print("successful go_Down call2")
        call_server6()
        print("successful gripAction call2")
        call_server1()
        print("successful goAhead call3")
        



if __name__ == '__main__':
    
    try:
        mastercontroller()
    except rospy.ROSInterruptException:
        print("error in the servers")
