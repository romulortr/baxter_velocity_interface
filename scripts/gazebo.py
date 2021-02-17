#!/usr/bin/python
import numpy as np
from scipy.spatial.transform import Rotation as scipy

import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

import filter

# @TODO read these parameters from rosparam server

MAX_PROPORTIONAL_GAIN = .01
MIN_PROPORTIONAL_GAIN = .001
INTEGRATOR_ANTI_WINDUP_VALUE = .01

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

    # Controller parameters
    self._proportional_gain = .01*np.ones(self._nb_joints)
    self._integrator =np.zeros(self._nb_joints)
    
    # Estimator parameters
    self._velocity_filter = filter.FIRFilter(filter='mean', nb_samples=10, dim=7) 

    # internal varialbes
    self._current_joint_velocity = np.zeros(self._nb_joints)
    self._desired_joint_velocity = np.zeros(self._nb_joints)
    self._raw_joint_position = np.zeros(self._nb_joints)
    self._link_desired_cartesian_velocity = np.zeros(6)
    self._prev_timestamp = 0

    # Publishers and subscribers
    self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
    self._odom_pub = rospy.Publisher('robot/odom', Odometry, queue_size=10)

    # Enabling robot
    print("Getting robot state... ")
    self._robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
    self._init_state = self._robot_state.state().enabled
    print("Enabling robot... ")
    self._robot_state.enable()
    self._pub_rate.publish(self._control_rate)
    self.set_neutral()

    print("Ready to control in velocity... ")
    self.control_timer = rospy.Timer(rospy.Duration(1./self._control_rate), self.control_callback)
    rospy.Subscriber('/robot/joint_states', JointState, self.joint_callback)
    rospy.Subscriber("robot/cmd_vel", Twist, self.twist_callback)

  def twist_callback(self, data):
    """
    Stores the desired cartesian velocity (link frame)
    """
    self._link_desired_cartesian_velocity = np.array([data.linear.x,
                                                        data.linear.y,
                                                        data.linear.z,
                                                        data.angular.x,
                                                        data.angular.y,
                                                        data.angular.z])
 
                                  
  def control_callback(self, event):
    """
    Computes and sends virtual waypoint in joint space
    """
    # Compute virtual waypoint to follow  desired joint velocity
    delta = (self._desired_joint_velocity) * self._proportional_gain + self._integrator
    # Send commands to robot using interface
    joint_command = {joint_name: self._raw_joint_position[i] + delta[i] for i, joint_name in enumerate(self._joints_name) }
    self._limb.set_joint_positions(joint_command)
    
  def joint_callback(self, data):     
    """
    Estimates joint velocities and updates control parameters.
    """  
    joint_position = dict(zip(data.name, data.position))
    raw_joint_position = np.zeros(self._nb_joints)
    joints_successfully_read = 0

    for i, joint_name in enumerate(self._joints_name):
      if joint_name in data.name:
        raw_joint_position[i] = joint_position[joint_name]
        joints_successfully_read += 1

    if joints_successfully_read == self._nb_joints:
      # Estimation 
      timestamp = data.header.stamp.secs + data.header.stamp.nsecs*(1e-9)    
      dt = timestamp - self._prev_timestamp
      raw_joint_velocity = (raw_joint_position - self._raw_joint_position)/dt
      self._current_joint_velocity = self._velocity_filter.filter(raw_joint_velocity)
      self._raw_joint_position = raw_joint_position
      self._prev_timestamp = timestamp
      
      desired_joint_position = np.array([0.,0.,0.,np.pi/4, 0, np.pi/2, 0])
      self._desired_joint_velocity = desired_joint_position - raw_joint_position
      self._desired_joint_velocity = np.minimum(np.maximum(self._desired_joint_velocity, -.1), .1)
      for i in range(self._nb_joints):
        error = self._desired_joint_velocity[i] - raw_joint_velocity[i]
        if error*self._desired_joint_velocity[i]  > 0:  
          self._proportional_gain[i] *= .9
        elif error*self._desired_joint_velocity[i]  < 0:
          self._proportional_gain[i] *= 1.05
        self._integrator[i] +=  2*self._proportional_gain[i]*error
      self._proportional_gain = np.minimum(np.maximum(self._proportional_gain, MIN_PROPORTIONAL_GAIN),
                                           MAX_PROPORTIONAL_GAIN)        
      self._integrator = np.minimum(np.maximum(self._integrator, -INTEGRATOR_ANTI_WINDUP_VALUE), 
                                    INTEGRATOR_ANTI_WINDUP_VALUE)

      # Control feedback
      self.velocity_callback()      

  def velocity_callback(self):
    """
    Transforms the desired cartesian velocity (link frame) to desired joint velocity 
    and computes an estimate for the current cartesian velocity (link frame).
    """
    ## Control loop
    # Transforming desired cartesian velocity to joint space 
    pose = self.kin.forward_position_kinematics()
    base_frame_desired_velocity = self.transform_velocity(
                                    pose,
                                    self._link_desired_cartesian_velocity, 
                                    inv=False)
    jacobian = np.asarray(self.kin.jacobian())
    inverse_jacobian = np.linalg.pinv(jacobian)
    self._desired_joint_velocity = np.matmul(inverse_jacobian, base_frame_desired_velocity)
    ## Estimation loop
    # Coompute current velocity in camera coordinate sytem
    base_frame_current_cartesian_velocity = np.matmul(jacobian, self._current_joint_velocity)
    camera_frame_current_cartesian_velocity = self.transform_velocity(
                                    pose,
                                    base_frame_current_cartesian_velocity, 
                                    inv=True)
    # Publishing velocity estimation
    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = camera_frame_current_cartesian_velocity[0]
    odom_msg.twist.twist.linear.y = camera_frame_current_cartesian_velocity[1] 
    odom_msg.twist.twist.linear.z = camera_frame_current_cartesian_velocity[2]
    odom_msg.twist.twist.angular.x = camera_frame_current_cartesian_velocity[3]
    odom_msg.twist.twist.angular.y = camera_frame_current_cartesian_velocity[4]
    odom_msg.twist.twist.angular.z = camera_frame_current_cartesian_velocity[5]
    self._odom_pub.publish(odom_msg)   

  def transform_velocity(self, pose, velocity, inv=False):
    trans = pose[0:3]
    quat = pose[3:]

    rotation = scipy.from_quat(quat)
    vel_ang = rotation.apply(velocity[3:], inverse=inv) 
    if inv:
      trans = -rotation.apply(trans, inverse=True)
    vel_lin = rotation.apply(velocity[0:3], inverse=inv) + np.cross(trans, vel_ang) 
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
    self.control_timer.shutdown()

    #return to normal
    self._reset_control_modes()
    #self.set_neutral()
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
