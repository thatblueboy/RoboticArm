#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import CVAction,goHomeAction,SweepAction,setTopAction,goAheadAction, goDownAction,gripAction
import robotic_arm_quark.msg
# refer server.txt for each server functionality
def call_CVserver():
    client = actionlib.SimpleActionClient('cv_actionServer', robotic_arm_quark.msg.CVAction)
    client.wait_for_server()
    client.wait_for_result()
    #result = client.getresult()   then return result 
    return 0



def call_server1():
    client = actionlib.SimpleActionClient('go_home', robotic_arm_quark.msg.goHomeAction)
    client.wait_for_server()
    client.wait_for_result()  
    #result = client.getresult()   then return result 
    return 0



def call_server2():
    client = actionlib.SimpleActionClient('sweep', robotic_arm_quark.msg.SweepAction)
    client.wait_for_server()
    client.wait_for_result()  
    #result = client.getresult()   then return result  
    return 0



def call_server3():
    client = actionlib.SimpleActionClient('set_Top', robotic_arm_quark.msg.setTopAction)
    client.wait_for_server()
    client.wait_for_result()  
    #result = client.getresult()   then return result 
    return 0



def call_server4():
    client = actionlib.SimpleActionClient('go_Ahead', robotic_arm_quark.msg.goAheadAction)
    client.wait_for_server()
    client.wait_for_result()  
    #result = client.getresult()   then return result 
    return 0



def call_server5():
    client = actionlib.SimpleActionClient('go_Down', robotic_arm_quark.msg.goDownAction)
    client.wait_for_server()
    client.wait_for_result()  
    #result = client.getresult()   then return result 
    return 0



def call_server6():
    client = actionlib.SimpleActionClient('grip', robotic_arm_quark.msg.gripAction)
    client.wait_for_server()
    client.wait_for_result()  
    #result = client.getresult()   then return result  
    return 0


if __name__ == '__main__':
    try:
        rospy.init_node('master_node1', anonymous = False)
        result = call_server1()
        print("successful")
        rospy.init_node('master_node2', anonymous = False)
        result = call_server2()
        print("successful")
    except rospy.ROSInterruptException:
        print("program interrupted before completion - call_server1() - goHomeServer or call_server2() - sweep_server ")


    try:
        rospy.init_node('master_node3', anonymous = False)
        result = call_server3()
        print("successful")
        rospy.init_node('master_node4', anonymous = False)
        result = call_server4()
        print("successful")
    except rospy.ROSInterruptException:
        print("program interrupted before completion - call_server3() - set_top_Server 0r call_server4() - goAhead_Server")


    try:
        rospy.init_node('cv_node', anonymous = False)
        result = call_server5()
        print("successful")
        rospy.init_node('cv_node', anonymous = False)
        result = call_server6()
        print("successful")
    except rospy.ROSInterruptException:
        print("program interrupted before completion - call_server5() - goDown_server or call_server6() - grip_server")
     # once picked drop to initial point

    try:
        result = call_server1()
        print("successful")
        result = call_server5()
        print("successful")
        result = call_server6()
        print("successful")
        result = call_server1()
        print("successful")
    except rospy.ROSInterruptException:
        print("program interrupted before completion - call_server1() - goHomeServer call2  or call_server5() - goDown_server call2 or call_server6() call 2 - grip_server or call_server1() - goHomeServer call3")

