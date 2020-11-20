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

class VelocityInterface:
  def __init__(self):
    self.kin = baxter_kinematics('right') 
    #
    self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
    self._limb = baxter_interface.limb.Limb("right")
    self._joint_names = self._limb.joint_names()
    self.nb_joints = len(self._limb.joint_names())

    # control parameters
    self._control_rate = 500.0  # Hz
    self._filter_rate = 50.0  # Hz

    print("Getting robot state... ")
    self._robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
    self._init_state = self._robot_state.state().enabled
    print("Enabling robot... ")
    self._robot_state.enable()
    self._pub_rate.publish(self._control_rate)
    print("Moving to neutral..")
    self.set_neutral()
    print("Ready to control in velocity... ")

    self.position =  np.zeros(self.nb_joints)
    self.gain = .01*np.ones(self.nb_joints)
    self.integrator = np.zeros(self.nb_joints)
    self.desired_joint_velocity = 0.*np.ones(self.nb_joints)
    self.direction = np.zeros(self.nb_joints)
    self.is_filter_active = False

    # set joint state publishing to 500Hz
    self.control_timer = rospy.Timer(rospy.Duration(1./self._control_rate), self.control_callback)
    self.filter_timer = rospy.Timer(rospy.Duration(1./self._filter_rate), self.filter_callback)    

  # Send linear and angular velocity (this works at image rate)
  def twist_callback(self, data):
    pass
        
  # Estimate linear and angular velocity of the end-effector
  def control_callback(self, event):
    # Most recent measurement
    joint_position = [self._limb.joint_angle(joint_name) for joint_name in self._joint_names]
    # Compute virtual waypoint to follow  desired joint velocity
    mean_gain = np.mean(self.gain)
    delta = (self.desired_joint_velocity + self.integrator  ) * self.gain
    # Send to robot using interface
    joint_command = {joint_name: joint_position[i] + delta[i] for i, joint_name in enumerate(self._joint_names) }
    self._limb.set_joint_positions(joint_command)
    
    """
    if joint_position > 1. and self.direction == 1:
      self.direction = -1
    elif joint_position < -1. and self.direction == -1:
      self.direction = 1
    delta = self.gain * self.direction 
    joint_command = {joint_name: joint_position + delta}
    self._limb.set_joint_positions(joint_command)
    """

  def filter_callback(self, event):
    joint_position = np.array([self._limb.joint_angle(joint_name) for joint_name in self._joint_names])
    if self.is_filter_active is True:   
      dt = (1./self._filter_rate)
      current_velocity = (joint_position - self.previous_joint_position)/dt
      for i in range(self.nb_joints):
        if self.desired_joint_velocity[i] - current_velocity[i] > 0:  
          self.gain[i] *= .9 
        else:
          self.gain[i] *= 1.05
        self.integrator[i] += .01*(self.desired_joint_velocity[i] - current_velocity[i])
        self.integrator[i] = min(max(self.integrator[i], -.1), .1)
      print(self.integrator)

    else:
      self.is_filter_active = True
    # saving for next iteration
    self.previous_joint_position = joint_position


   
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
    print("\nExiting example...")
    # Shutdown timer callback
    self.control_timer.shutdown()
    self.filter_timer.shutdown()    
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
