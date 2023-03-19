import roslib
import rospy
from geometry_msgs.msg import Point

roslib.load_manifest('my_pkg_name')
import actionlib
from std_msgs.msg import Bool, Float64, Float64MultiArray

from robotic_arm_quark.msg import visualServeyAction

# subscribe to /feedback coming from camera , this provides x and y of contour appearing in frame. apply individual pids for
# x and y, setpoints would becentre of cameraframe(rosparam has been made for the same) no need to publish anywhere, that is 
# handled by the pid package.
# if both x and y are within a error range from centre of frame, service success, turn off pids,job done


class visualSurvey_server:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('visualSurvey_server', visualServeyAction, self.execute, False)
    self.server.start()
    self.goalpose = [0, 0, 0, 0, 0]# to be defined
  
  def execute(self, goal):
      Controller()

class Controller:
  def __init__(self,array):

      enable = Bool()
      enable = True
      self.target_x=0
      self.target_y=0
      self.v=0
      self.sub=rospy.Subscriber('/feedback',Float64MultiArray,self.callback)
      self.velocity=rospy.Subscriber('/control_effort',Float64,self.Vel)

      self.enableMotor1 = rospy.Publisher('/motor1/pid_enable', Bool, queue_size=10)
      self.enableMotor2 = rospy.Publisher('/motor2/pid_enable', Bool, queue_size=10)
      self.stateArray = [0, 0, 0, 0, 0]

      self.theta1 = Float64()
      self.theta2 = Float64()


      self.theta1 = array[0]
      self.theta2 = array[1]


      self.goalArray = [self.theta1,self.theta2,]

      # start PIDs
      self.enableMotor1.publish(enable)
      self.enableMotor2.publish(enable)


      self.setpointPublisherMotor1 = rospy.Publisher(
          '/motor1/setpoint', Float64, queue_size=10)
      self.setpointPublisherMotor2 = rospy.Publisher(
          '/motor2/setpoint', Float64, queue_size=10)

      self.statePublisherMotor1 = rospy.Publisher(
          '/motor1/state', Float64, queue_size=10)
      self.statePublisherMotor2 = rospy.Publisher(
          '/motor2/state', Float64, queue_size=10)


      while not rospy.is_shutdown:
          rospy.Subscriber('/feedback', Float64MultiArray, self.sendToPID)
          if self.reached():
              enable = False

              # terminate PIDs
              self.enableMotor1.publish(enable)
              self.enableMotor2.publish(enable)

              # exit from class
              break

  def sendToPID(self, array):
      # publish state(curent angles)
      self.state = array
      self.statePublisherMotor1.publish(array[0])
      self.statePublisherMotor2.publish(array[1])


      # publish goal
      self.setpointPublisherMotor1.publish(self.goalArray[0])
      self.setpointPublisherMotor2.publish(self.goalArray[0])


  def reached(self):
      if abs(self.stateArray[0] - self.goalArray[0]) < 0.4 and abs(self.stateArray[1] - self.goalArray[1] < 0.4) and abs(self.stateArray[2] - self.goalArray[2])<0.4 and abs(self.stateArray[3] - self.goalArray[3])<0.4 and abs(self.stateArray[0] - self.goalArray[0]):
          return True


  def callback(self,coords):
    self.object_x=coords[0]
    self.object_y=coords[1]

  def Vel(self,v):
    self.velocity=v

  while not rospy.is_shutdown:
    if self.reached():
        enable = False

        # terminate PIDs
        self.enableMotor1.publish(enable)
        self.enableMotor2.publish(enable)
        # exit from class
        break




if __name__ == '__main__':
  rospy.init_node('sweep_server')
  server = visualSurvey_server()
  rospy.spin()
