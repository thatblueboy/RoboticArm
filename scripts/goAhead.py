from robotic_arm_quark.msg import goAheadAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
from math import tan, acos, asin, cos, sin, atan
roslib.load_manifest('robotic_arm_quark')

# take input from CV on /feedback, in callback, publish a constant value(r(radial velocity, as float64)) to a topic
# once a contour is found in feed and it has travelled sufficient distance into the frame, publish velocity as 0, service successful

class goAhead_Server:
    def __init__(self, cmd_vel):
        self.server = actionlib.SimpleActionServer(
            'go_Ahead', goAheadAction, self.execute, False)
        self.server.start()
        
        self.sub2 = rospy.Subscriber('/motor2/state', Float64, self.callback1)
        self.sub3 = rospy.Subscriber('/motor3/state', Float64, self.callback2)

        self.pub2 = rospy.Publisher('/motor2/pwm', Float64, queue_size=1)
        self.pub3 = rospy.Publisher('/motor3/pwm', Float64, queue_size=1)

        self.pose = [0, 0, 0, 0, 0]

        rospy.spin()

    def execute(self, goal):
        goAhead()

       
       
        self.server.set_succeeded()

class goAhead():
    pass


if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = goAhead_Server()
    rospy.spin()
