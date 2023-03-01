import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import roslib
roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import setTopAction



class sweep_server:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('set_Top', setTopAction, self.execute, False)
    self.server.start()

    self.pose = [0, 0, 0, 0, 0]

#   def execute(self, goal):
#     self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = setTopServer()
  rospy.spin()
