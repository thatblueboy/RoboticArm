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


if __name__ == '__main__':
  rospy.init_node('sweep_server')
  server = visualSurvey_server()
  rospy.spin()
