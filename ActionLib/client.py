#! /usr/bin/env python3

import rospy

import actionlib

from robotic_arm_quark.msg import FibonacciAction, FibonacciGoal


def fibonacci_client():
    
    client = actionlib.SimpleActionClient(
        'fibonacci', FibonacciAction)


    print('waiting for server')
    client.wait_for_server()

   
    goal = FibonacciGoal()
    goal.order = 20
 
    client.send_goal(goal)
    print("goal sent")

    
    client.wait_for_result()
    
    return client.get_result() 


if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))

    except rospy.ROSInterruptException:
        pass
