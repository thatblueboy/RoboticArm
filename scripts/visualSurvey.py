import rospy
from geometry_msgs.msg import Point
import roslib
roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import visualServeyAction
from std_msgs.msg import Float64MultiArray, Bool, Float64



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


if __name__ == '__main__':
  rospy.init_node('sweep_server')
  server = visualSurvey_server()
  rospy.spin()
