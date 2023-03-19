from robotic_arm_quark.msg import goAheadAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray,Float64, Bool
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
        
        self.pose = [0, 0, 0, 0, 0]

        rospy.spin()

    def execute(self, goal):
        l1=1
        l2=2
        goAhead(l1,l2)
       
        self.server.set_succeeded()

class goAhead():
    def __init__(self,l1,l2):
        self.enable = Bool()
        self.enable = False
        self.l1=l1
        self.l2=l2
        self.theta1=0
        self.theta2=0
        self.sub2 = rospy.Subscriber('/frame', Float64, self.callback1)
        self.pub2 = rospy.Publisher('/motor2/pwm', Float64, queue_size=1)
        self.pub3 = rospy.Publisher('/motor3/pwm', Float64, queue_size=1)

        #FK
        # self.x=cos(self.theta1)+cos(self.theta2)
        # self.y=sin(self.theta1)+sin(self.theta2)

        # self.setpointPublisherMotor1 = rospy.Publisher(
        #     '/motor1/setpoint', Float64, queue_size=10)
        # self.setpointPublisherMotor2 = rospy.Publisher(
        #     '/motor2/setpoint', Float64, queue_size=10)

        # self.statePublisherMotor1 = rospy.Publisher(
        #     '/motor1/state', Float64, queue_size=10)
        # self.statePublisherMotor2 = rospy.Publisher(
        #     '/motor2/state', Float64, queue_size=10)


        while not rospy.is_shutdown:
            if self.reached():
                # exit from class
                break
    
    def callback1(self,data):
        self.frame=data

    def calculate(self):
        #IK
        #if out of frame
        if self.frame:
            self.theta2 = acos(((self.x*self.x)+(self.y*self.y)-(self.l2*self.l2)-(self.l1*self.l1))/(2*self.l1*self.l2))
            self.theta1 = atan(self.y/self.x)-atan(self.l2*sin(self.theta2)/(self.l1+self.l2*cos(self.theta2)))
            self.x+=1
        else:
            self.reached()
    


    def reached(self):
        
            return True


if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = goAhead_Server()
    rospy.spin()
