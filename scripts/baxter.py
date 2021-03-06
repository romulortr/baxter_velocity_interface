#!/usr/bin/python
import numpy as np
import random
import math
from numpy.testing._private.utils import measure
from scipy.spatial.transform import Rotation as scipy

import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from baxter_core_msgs.msg import JointCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

import joint_filter

class VelocityInterface:
  def __init__(self):
    # PyKDL
    print("Initializing pyKDL")
    self.kin = baxter_kinematics('right') 
    self._limb = baxter_interface.limb.Limb("right")
    self._joints_name = self._limb.joint_names()
    self._nb_joints = len(self._limb.joint_names())
    print("Success!")
    print('-----------------')

    # control parameters
    self._control_rate = 500.0  # Hz (This has to be as fast as possible)
    self._estimation_rate = 100.0

    # Estimator parameters
    self._velocity_filter = joint_filter.FIRFilter(filter='mean', nb_samples=5)  
    # internal varialbes
    self._current_joint_velocity = np.zeros(self._nb_joints)
    self._desired_joint_velocity = np.zeros(self._nb_joints)
    self._camera_desired_cartesian_velocity = np.zeros(6)

    # Publishers and subscribers
    self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
    self._odom_pub = rospy.Publisher('robot/odom', Odometry, queue_size=10)
    rospy.Subscriber('/robot/joint_states', JointState, self.joint_callback)
    rospy.Subscriber("robot/cmd_vel", Twist, self.twist_callback)

    # Enabling robot
    print("Getting robot state... ")
    self._robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
    self._init_state = self._robot_state.state().enabled
    print("Enabling robot... ")
    self._robot_state.enable()
    self._pub_rate.publish(self._control_rate)
    self.set_neutral()

    print("Ready to control in velocity... ")
    self._control_timer = rospy.Timer(rospy.Duration(1./self._control_rate), self.control_callback)
    self._estimation_timer = rospy.Timer(rospy.Duration(1./self._estimation_rate), self.estimation_callback)

  def twist_callback(self, data):
    """
    Stores the desired cartesian velocity (link frame)
    """
    self._camera_desired_cartesian_velocity = np.array([data.linear.x,
                                                     data.linear.y,
                                                     data.linear.z,
                                                     data.angular.x,
                                                     data.angular.y,
                                                     data.angular.z])
 
                                  
  def control_callback(self, event):
    """
    Computes and sends virtual waypoint in joint space
    """
    # Prepare message to publish
    joint_command = {joint_name: self._desired_joint_velocity[i]  for i, joint_name in enumerate(self._joints_name) }
    self._limb.set_joint_velocities(joint_command)
    
  def joint_callback(self, data):     
    """
    Estimates joint velocities and updates control parameters.
    """  
    if len(data.name) > 10:
      joint_velocity = dict(zip(data.name, data.velocity))
      raw_joint_velocity = np.zeros(self._nb_joints)
      for i, joint_name in enumerate(self._joints_name):
        if joint_name in data.name:
          raw_joint_velocity[i] = joint_velocity[joint_name]

      self._current_joint_velocity = self._velocity_filter.filter(raw_joint_velocity)
 

  def velocity_callback(self, event):
    """
    Transforms the desired cartesian velocity (link frame) to desired joint velocity 
    and computes an estimate for the current cartesian velocity (link frame).
    """
    ## Computing desired joint velocity
    pose = self.kin.forward_position_kinematics()
    base_frame_desired_velocity = self.transform_velocity(
                                    pose,
                                    self._camera_desired_cartesian_velocity, 
                                    inv=False)
    jacobian = np.asarray(self.kin.jacobian())
    jacobian_inverse = np.linalg.pinv(jacobian)
    self._desired_joint_velocity = np.matmul(jacobian_inverse, base_frame_desired_velocity)
 
    ## Publish camera velocity
    # Coompute current velocity in camera coordinate sytem
    base_frame_current_cartesian_velocity = np.matmul(jacobian, self._current_joint_velocity)
    camera_frame_current_cartesian_velocity = self.transform_velocity(
                                    pose,
                                    base_frame_current_cartesian_velocity, 
                                    inv=True)
    
    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = camera_frame_current_cartesian_velocity[0]
    odom_msg.twist.twist.linear.y = camera_frame_current_cartesian_velocity[1] 
    odom_msg.twist.twist.linear.z = camera_frame_current_cartesian_velocity[2]
    odom_msg.twist.twist.angular.x = camera_frame_current_cartesian_velocity[3]
    odom_msg.twist.twist.angular.y = camera_frame_current_cartesian_velocity[4]
    odom_msg.twist.twist.angular.z = camera_frame_current_cartesian_velocity[5]
    self._odom_pub.publish(odom_msg)   

  def transform_velocity(self, pose, velocity, inv=False):
    trans = pose[0:3] # np.array([0.038, 0.012, -0.142])
    quat = pose[3:]

    rotation = scipy.from_quat(quat)
    vel_ang = rotation.apply(velocity[3:], inverse=inv) 
    if inv:
      trans = -rotation.apply(trans, inverse=True)
    vel_lin = rotation.apply(velocity[0:3], inverse=inv) #+ np.cross(trans, vel_ang) 
    return np.hstack([vel_lin, vel_ang])

  def set_neutral(self):
    """
    Sets arm back into a neutral pose.
    """
    print("Moving to neutral pose...")
    self._limb.move_to_neutral()

  def _reset_control_modes(self):
    rate = rospy.Rate(self._control_rate)
    for _ in xrange(100):
      if rospy.is_shutdown():
        return False
      self._limb.exit_control_mode()
      self._pub_rate.publish(100)  # 100Hz default joint state rate
      rate.sleep()
    return True

  def clean_shutdown(self):
    print("\nExiting velocity interface...")
    # Shutdown timer callback
    self._control_timer.shutdown()
    self._estimation_timer.shutdown()

    #return to normal
    self._reset_control_modes()
    self.set_neutral()
    if not self._init_state:
      print("Disabling robot...")
      self._robot_state.disable()
    return True

if __name__ == "__main__":
  print("Initializing node... ")
  rospy.init_node("baxter_velocity_interface")
  vel_interface = VelocityInterface()

  rospy.on_shutdown(vel_interface.clean_shutdown)
  rospy.spin()
  print("Done.")
