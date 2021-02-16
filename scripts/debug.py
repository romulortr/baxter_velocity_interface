#!/usr/bin/python
import numpy as np
from scipy.spatial.transform import Rotation as scipy

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import filter

class VelocityInterface:
  def __init__(self):
    self.nb_filters = 1
    self._velocity_filter = [filter.FIRFilter(filter='mean', nb_samples=30*i+1, dim=6) for i in range(0,self.nb_filters)]

    # Publishers and subscribers
    self._odom_raw_pub = rospy.Publisher('robot/odom/raw', Odometry, queue_size=10)
    rospy.Subscriber("robot/cmd_vel", Twist, self.twist_callback)
    rospy.Subscriber("robot/odom", Odometry, self.odom_callback)

    # Variables
    self.velocity= np.zeros(12)

  def twist_callback(self, data):
    self.velocity[6:] = np.array([
                        data.linear.x,
                        data.linear.y,
                        data.linear.z,
                        data.angular.x,
                        data.angular.y,
                        data.angular.z])
  
  def odom_callback(self, data):
    self.velocity[:6] = np.array([
                        data.twist.twist.linear.x,
                        data.twist.twist.linear.y,
                        data.twist.twist.linear.z,
                        data.twist.twist.angular.x,
                        data.twist.twist.angular.y,
                        data.twist.twist.angular.z])
    print(' '.join(map(str, self.velocity)))

if __name__ == "__main__":
  print("Initializing node... ")
  rospy.init_node("baxter_velocity_interface")
  vel_interface = VelocityInterface()

  rospy.spin()
  print("Done.")
