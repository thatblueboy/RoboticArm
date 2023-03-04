import rospy
from geometry_msgs.msg import Point
import roslib
roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import SweepAction
from std_msgs.msg import Float64MultiArray, Bool, Float64



class sweep_server:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('sweep', SweepAction, self.execute, False)
    self.server.start()
    self.goalpose = [0, 0, 0, 0, 0]# to be defined
  
  def execute(self, goal):
      Controller()

class Controller:
   def __init__(self,array):
      enable = Bool()
      enable = True

      self.stateArray = [0, 0, 0, 0, 0] 

      self.theta1 = Float64()
      self.theta2 = Float64()
      self.theta3 = Float64()
      self.theta4 = Float64()
      self.theta5 = Float64()

      self.theta1 = array[0]
      self.theta2 = array[1]
      self.theta3 = array[2]
      self.theta4 = array[3]
      self.theta5 = array[4]

      self.goalArray = [self.theta1,
                          self.theta2,
                          self.theta3,
                          self.theta4,
                          self.theta5]
      
      self.enableMotor1 = rospy.Publisher(
            '/motor1/pid_enable', Bool, queue_size=10)
    

      self.enableMotor1.publish(enable)
      
      self.setpointPublisherMotor1 = rospy.Publisher(
            '/motor5/setpoint', Float64, queue_size=10)

      self.statePublisherMotor1 = rospy.Publisher(
            '/motor5/state', Float64, queue_size=10)


      while not rospy.is_shutdown:
            rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID)
            if self.reached():
                enable = False
                # terminate PIDs
                self.enableMotor5.publish(enable)

                #exit from class
                break
   def sendToPID(self, array):
        # publish state(curent angles)
        self.state = array
        self.statePublisherMotor1.publish(array[0])
        # publish goal
        self.setpointPublisherMotor1.publish(self.goalArray[0])
   def reached(self):  
      if abs(self.stateArray[0] - self.goalArray[0]) < 0.4 :
            return True 


if __name__ == '__main__':
  rospy.init_node('sweep_server')
  server = sweep_server()
  rospy.spin()
