#!/usr/bin/python
import numpy as np
import random
import math
from scipy.spatial.transform import Rotation as scipy

import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from baxter_core_msgs.msg import JointCommand

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
class VelocityInterface:
  def __init__(self):
    self.kin = baxter_kinematics('right') 
    #
    self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, self.twist_callback)

    self._limb = baxter_interface.limb.Limb("right")
    self._joint_names = self._limb.joint_names()
    self.nb_joints = len(self._limb.joint_names())

    # control parameters
    self._control_rate = 500.0  # Hz
    self._filter_rate = 50.0  # Hz
    self._velocity_rate = 25 # Hz (This should be equal or faster than the rate of twist message) 
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
    self.desired_velocity = np.zeros(6)
    self.current_velocity = np.zeros(7)
    self.direction = np.zeros(self.nb_joints)
    self.is_filter_active = False
    
    # set joint state publishing to 500Hz
    self.control_timer = rospy.Timer(rospy.Duration(1./self._control_rate), self.control_callback)
    self.filter_timer = rospy.Timer(rospy.Duration(1./self._filter_rate), self.filter_callback)    
    self.velocity_timer = rospy.Timer(rospy.Duration(1./self._velocity_rate), self.velocity_callback)    

  def twist_callback(self, data):
    """
    Stores the desired cartesian velocity (link frame)
    """
    self.desired_velocity = np.array([data.linear.x,
                                      data.linear.y,
                                      data.linear.z,
                                      data.angular.x,
                                      data.angular.y,
                                      data.angular.z])
 
                                  
  # Estimate linear and angular velocity of the end-effector
  def control_callback(self, event):
    """
    ADD A DESCRIPTION HERE
    """
    # Most recent measurement
    joint_position = [self._limb.joint_angle(joint_name) for joint_name in self._joint_names]
    # Compute virtual waypoint to follow  desired joint velocity
    mean_gain = np.mean(self.gain)
    delta = (self.desired_joint_velocity ) * self.gain + self.integrator
    # Send to robot using interface
    joint_command = {joint_name: joint_position[i] + delta[i] for i, joint_name in enumerate(self._joint_names) }
    self._limb.set_joint_positions(joint_command)
    
  def filter_callback(self, event):     
    """
    ADD A DESCRIPTION HERE
    """  
    # Estimate joint velocity
    joint_position = np.array([self._limb.joint_angle(joint_name) for joint_name in self._joint_names])
    if self.is_filter_active is True:   
      dt = (1./self._filter_rate)
      self.current_velocity = (joint_position - self.previous_joint_position)/dt
      for i in range(self.nb_joints):
        if self.desired_joint_velocity[i] - self.current_velocity[i] > 0:  
          self.gain[i] *= .9 
        else:
          self.gain[i] *= 1.05
        self.integrator[i] += .005*(self.desired_joint_velocity[i] - self.current_velocity[i])
        self.integrator[i] = min(max(self.integrator[i], -.1), .1)
        self.gain[i] = min(max(self.gain[i], .001), .1)        
    else:
      self.is_filter_active = True
    # saving for next iteration
    self.previous_joint_position = joint_position

  def velocity_callback(self, event):
    """
    Transforms the desired cartesian velocity (link frame) to desired joint velocity 
    and computes an estimate for the current cartesian velocity (link frame).
    """
    ## Control loop
    # Transforming desired cartesian velocity to joint space 
    quat = self.kin.forward_position_kinematics()[3:]
    rotation = scipy.from_quat(quat)
    base_frame_desired_velocity = np.hstack([
      rotation.apply(self.desired_velocity[0:3], inverse=False),
      rotation.apply(self.desired_velocity[3:], inverse=False)])
    jacobian = np.asarray(self.kin.jacobian())
    inverse_jacobian = np.linalg.pinv(jacobian)
    self.desired_joint_velocity = np.matmul(inverse_jacobian, base_frame_desired_velocity)
    
    ## Estimation loop
    # Coompute current velocity in link coordinate sytem
    base_frame_current_cartesian_velocity = np.matmul(jacobian, self.current_velocity)
    link_frame_current_cartesian_velocity = np.hstack([
      rotation.apply(base_frame_current_cartesian_velocity[0:3], inverse=True),
      rotation.apply(base_frame_current_cartesian_velocity[3:], inverse=True)
    ]) 

    # Write publisher
    print( link_frame_current_cartesian_velocity)

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
    self.velocity_timer.shutdown()    

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
