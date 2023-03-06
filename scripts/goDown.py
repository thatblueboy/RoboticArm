import roslib
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64

roslib.load_manifest('my_pkg_name')
import actionlib
from robotic_arm_quark.msg import goDownAction

#  take ik from data from a rostopic, assume this provides x, y in 2d vertical plane, take y as state for pid and publish to pid 
# in callback function. assume setpoint for y is a rosparam(pick_height)

class goDown_server:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('go_Down', goDownAction, self.execute, False)
    self.server.start()

    self.pose = [0, 0, 0, 0, 0]

  def execute(self, goal):
    # write code here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = goDown_server()
  rospy.spin()
