import math

import actionlib
import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import visualServeyAction

roslib.load_manifest('my_pkg_name')


# subscribe to /feedback coming from camera , this provides x and y of contour appearing in frame. apply individual pids for
# x and y, setpoints would becentre of cameraframe(rosparam has been made for the same) no need to publish anywhere, that is
# handled by the pid package.
# if both x and y are within a error range from centre of frame, service success, turn off pids,job done


class visualSurvey_server:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'visualSurvey_server', visualServeyAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        l1 = rospy.get_param('/link1')
        l2 = rospy.get_param('/link2')
        Controller(goal, l1, l2)


class Controller:
    def __init__(self, array, l1, l2):

        enable = Bool()
        enable = True
        self.target_x = 0
        self.target_y = 0
        self.v = 0
        self.theta2 = 0
        self.theta3 = 0
        self.l1 = l1
        self.l2 = l2
        self.dt = 0.5

        self.velocity = rospy.Subscriber(
            '/cvY/control_effort', Float64, self.VelLinear)
        self.velocity = rospy.Subscriber(
            '/cvX/control_effort', Float64, self.VelAngular)
        self.SubState2 = rospy.Subscriber(
            '/motor2/state', Float64, self.getState2)
        self.SubState3 = rospy.Subscriber(
            '/motor3/state', Float64, self.getState3)

        self.enableCVx = rospy.Publisher(
            '/cvX/pid_enable', Bool, queue_size=10)
        self.enableCVy = rospy.Publisher(
            '/cvY/pid_enable', Bool, queue_size=10)
        self.effortPublisherMotor1 = rospy.Publisher(
            '/motor1/control_effort', Float64, queue_size=10)
        self.effortPublisherMotor2 = rospy.Publisher(
            '/motor2/control_effort', Float64, queue_size=10)
        self.effortPublisherMotor3 = rospy.Publisher(
            '/motor3/control_effort', Float64, queue_size=10)

        self.stateArray = [0, 0]

        # FK
        self.x = rospy.get_param('/x')
        self.z = rospy.get_param('/y')

        self.centre_x = Float64()
        self.centre_y = Float64()

        self.centre_x = array[0]
        self.centre_y = array[1]

        self.goalArray = [self.centre_x, self.centre_y]

        # start PIDs
        self.enableCVx.publish(enable)
        self.enableCVy.publish(enable)

        self.setpointPublisherCVx = rospy.Publisher(
            '/cvX/setpoint', Float64, queue_size=10)
        self.setpointPublisherCVy = rospy.Publisher(
            '/cvY/setpoint', Float64, queue_size=10)

        self.statePublisherCVx = rospy.Publisher(
            '/cvX/state', Float64, queue_size=10)
        self.statePublisherCVy = rospy.Publisher(
            '/cvY/state', Float64, queue_size=10)

        while not rospy.is_shutdown:
            rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID1)
            if self.reached():
                enable = False

                # terminate PIDs
                self.enableCVx.publish(enable)
                self.enableCVy.publish(enable)

                # exit from class
                break

    def getState2(self, theta2):
        self.theta2 = theta2

    def getState3(self, theta3):
        self.theta3 = theta3

    def sendToPID1(self, coords):
        # publish state(curent angles)
        self.stateArray = coords
        self.statePublisherCVx.publish(coords[0])
        self.statePublisherCVy.publish(coords[1])

        # publish goal
        self.setpointPublisherCVx.publish(self.goalArray[0])
        self.setpointPublisherCVy.publish(self.goalArray[1])

        # calculate theta2,theta3
    def VelLinear(self, v):
        velocityRadial = v
        theta3 = math.acos(((self.x*self.x)+(self.z*self.z) -
                            (self.l2*self.l2)-(self.l1*self.l1))/(2*self.l1*self.l2))
        theta2 = math.atan(self.z/self.x)-math.atan(self.l2 *
                                                    math.sin(self.theta2)/(self.l1+self.l2*math.cos(self.theta2)))
        self.goTo(theta2, theta3)
        self.x += velocityRadial*self.dt

    def VelAngular(self, v):
        velocityTangential = v
        omega1 = velocityTangential/self.x  # v=wx
        self.effortPublisherMotor1.publish(omega1)

    def goTo(self, theta2, theta3):
        omega2 = (self.theta2-theta2)/self.dt
        omega3 = (self.theta3-theta3)/self.dt
        self.effortPublisherMotor2.publish(omega2)
        self.effortPublisherMotor3.publish(omega3)

    def reached(self):
        if abs(self.stateArray[0] - self.goalArray[0]) < 0.4 and abs(self.stateArray[1] - self.goalArray[1] < 0.4):
            self.effortPublisherMotor1.publish(0)
            self.effortPublisherMotor2.publish(0)
            self.effortPublisherMotor3.publish(0)

            return True


if __name__ == '__main__':
    rospy.init_node('visualSurvey')
    server = visualSurvey_server
