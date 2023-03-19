from robotic_arm_quark.msg import goAheadAction
import actionlib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
from math import tan, acos, asin, cos, sin, atan
roslib.load_manifest('robotic_arm_quark')

class goAhead_Server:
    def __init__(self, cmd_vel):
        self.server = actionlib.SimpleActionServer(
            'go_Ahead', goAheadAction, self.execute, False)
        self.server.start()
        self.theta1 = 0
        self.theta2 = 0
        self.l1 = 30
        self.l2 = 20
        self.InitError1 = 0
        self.InitError2 = 0
        # self.integral_linear=0
        self.v_effector = cmd_vel
        self.kp = 0.8
        self.kd = 0.8
        self.goalx = 5
        self.goaly = 5

        # self.x=0
        # self.y=5
        self.sub2 = rospy.Subscriber('/motor2/state', Float64, self.callback1)
        self.sub3 = rospy.Subscriber('/motor3/state', Float64, self.callback2)
        self.pub2 = rospy.Publisher('/motor2/pwm', Float64, queue_size=1)
        self.pub3 = rospy.Publisher('/motor3/pwm', Float64, queue_size=1)
        self.pose = [0, 0, 0, 0, 0]
        rospy.spin()

    def callback1(self, msg):
        self.theta1 = msg

    def callback2(self, msg):
        self.theta2 = msg
#   def execute(self, goal):

        def calculate(self):
            dt = 0.05
            x = self.goalx  # static for now
            y = self.goaly

        # BROTHER WHY DONT YOU HAVE AUTOSAVE, i do nibba
            # FK
            # current_x=cos(self.theta1)+cos(self.theta2)
            # current_y=sin(self.theta1)+sin(self.theta2)

            # IK

            if self.InitError2 >= 0:
                target_theta2 = acos(
                    ((x*self.x)+(y*y)-(self.l2*self.l2)-(self.l1*self.l1))/(2*self.l1*self.l2))
                target_theta1 = atan(
                    y/x)-atan(self.l2*sin(self.theta2)/(self.l1+self.l2*cos(theta2)))
            else:
                target_theta2 = -1 * \
                    acos(((x*self.x)+(y*y)-(self.l2*self.l2) -
                         (self.l1*self.l1))/(2*self.l1*self.l2))
                target_theta1 = atan(
                    y/x)+atan(self.l2*sin(theta2)/(self.l1+self.l2*cos(theta2)))

            error1 = self.theta1-target_theta1
            differential1 = (error1-self.InitError1)/dt
            error1 = self.theta1-target_theta1
            pwm1 = self.kp*error1+kd*differential1

            error2 = self.theta2-target_theta2
            differential2 = (error2-self.InitError2)/dt
            pwm2 = self.kp*error2+self.kd*differential2

            self.move(pwm1, pwm2)

            self.InitError1 = error1
            self.InitError2 = error2

        def move(self, pwm1, pwm2):
            # y_const
            # help with /cmd_vel!!!!!, will look into it 
            # assign prev errors

            # next x from linear vel
            # x=current_x+self.v_effector*self.dt
            self.pub2.publish(pwm1)
            self.pub3.publish(pwm2)

            # height correction
            # if current_y>=5.4:
            #     y=current_y-self.v_effector*self.dt
            # elif current_y<=4.6:
            #     y=current_y+self.v_effector*self.dt

#     self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('do_dishes_server')
    server = goAhead_Server()
    rospy.spin()
