#!/usr/bin/python
import numpy as np
import random
import math

import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt16

from baxter_core_msgs.msg import JointCommand

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics


#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ######
#              DO NOT USE IT, WORKING IN PROGRESS                      #
#### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ######

class VelocityInterface:
  def __init__(self):
    self.kin = baxter_kinematics('right') 

    #
    self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
    self._right_arm = baxter_interface.limb.Limb("right")
    self._right_joint_names = self._right_arm.joint_names()

    # control parameters
    self._rate = 500.0  # Hz

    print("Getting robot state... ")
    self._robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
    self._init_state = self._robot_state.state().enabled
    print("Enabling robot... ")
    self._robot_state.enable()
    self.set_neutral()
    # set joint state publishing to 500Hz
    self.start=rospy.Time.now()
    #rospy.Timer(rospy.Duration(1./self._rate), self.velocity_callback)
    print("Ready to control in velocity... ")
   

  # Send linear and angular velocity (this works at image rate)
  def twist_callback(self, data):
    pass
    
    desired_velocity = np.array([data.linear.x, 
                                 data.linear.y, 
                                 data.linear.z,
                                 data.angular.x, 
                                 data.angular.y, 
                                 data.angular.z]).reshape([6,1])
    
    
  # Estimate linear and angular velocity of the end-effector
  def velocity_callback(self, event):
    # Most recent measurement
    twist = self.kin.forward_velocity_kinematics()
    measured_velocity = np.array([twist.vel.x(),
                                  twist.vel.y(),
                                  twist.vel.z(),
                                  twist.rot.x(),
                                  twist.rot.y(),
                                  twist.rot.z()])
    # Update velocity filter
    current_velocity = self.velocity_filter(measured_velocity)

    # Compute control input in cartesian space
    desired_velocity = np.array([0,0,0.,0,0,0.0]).reshape([6,1])
    reference_velocity = self.velocity_controller(desired_velocity, desired_velocity)
    
    # Compute control input in joint space
    jacobian_inverse = self.kin.jacobian_pseudo_inverse()
    desired_joint_velocity = np.matmul(jacobian_inverse, reference_velocity)

    # Prepare message to publish
    desired_joint_velocity[-1,0] = 1.
    cmd = dict([(joint, desired_joint_velocity[i,0]) for i, joint in enumerate(self._right_joint_names)])
    #print(cmd)
    #print(desired_joint_velocity.flatten())
    self._right_arm.set_joint_velocities(cmd)
    #print(measured_velocity)

  # Filter to remove noise  
  def velocity_filter(self, measured_velocity, current_velocity=None):
    # Some fancy filter here..
    filtered_velocity = 1.*measured_velocity.reshape([6,1])  
    return filtered_velocity
  
  # Compute (linear and angular) velocity reference
  def velocity_controller(self, desired_velocity, current_velocity=None):
    # Some fancy control law here..
    reference_velocity = 1.*desired_velocity.reshape([6,1])
    return reference_velocity

  def set_neutral(self):
    """
    Sets arm back into a neutral pose.
    """
    print("Moving to neutral pose...")
    self._right_arm.move_to_neutral()

  def _reset_control_modes(self):
    rate = rospy.Rate(self._rate)
    for _ in xrange(100):
      if rospy.is_shutdown():
        return False
      self._right_arm.exit_control_mode()
      self._pub_rate.publish(100)  # 100Hz default joint state rate
      rate.sleep()
    return True

  def clean_shutdown(self):
    print("\nExiting example...")
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
  vel_interface.wobble()
  #rospy.spin()
  print("Done.")
