#!/usr/bin/env python
import time
import numpy as np
from scipy.spatial.transform import Rotation as scipy
from matplotlib import pyplot as plt

import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
import baxter_external_devices

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class KeyboardTeleop:
  def __init__(self):
    self.kin = baxter_kinematics('right') 
    rate = 20 # hz 
    self.base_frame_desired_cartesian_velocity = np.zeros(6)
 
    self.twist_pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('robot/odom', Odometry, self.odom_callback)
    rospy.Timer(rospy.Duration(1./rate), self.twist_publisher)  

    self.log_file = open("log_file.txt", "w")

    self.map_keyboard()

  def twist_publisher(self, event):
    # Transform from base to link frame
    quat = self.kin.forward_position_kinematics()[3:]
    rotation = scipy.from_quat(quat)
    link_frame_desired_cartesian_velocity = np.hstack([
      rotation.apply(self.base_frame_desired_cartesian_velocity[0:3], inverse=True),
      rotation.apply(self.base_frame_desired_cartesian_velocity[3:], inverse=True)])   

    # Write message and send it
    twist = Twist()
    twist.linear.x = link_frame_desired_cartesian_velocity[0]
    twist.linear.y = link_frame_desired_cartesian_velocity[1]
    twist.linear.z = link_frame_desired_cartesian_velocity[2]
    twist.angular.x = link_frame_desired_cartesian_velocity[3]
    twist.angular.y = link_frame_desired_cartesian_velocity[4]
    twist.angular.z = link_frame_desired_cartesian_velocity[5]

    self.twist_pub.publish(twist)

  def update_desired_velocity(self, field, value):
    self.base_frame_desired_cartesian_velocity[field] = value

  def map_keyboard(self):
    lin_vel = .02
    ang_vel = .05
    bindings = {
    #   key: (function, args, description)
        'q': (self.update_desired_velocity, [0, lin_vel], "linear_x_positive"),
        'a': (self.update_desired_velocity, [0, -lin_vel], "linear_x_negative"),
        'z': (self.update_desired_velocity, [0, 0], "linear_x_stop"),
        'w': (self.update_desired_velocity, [1, lin_vel], "linear_y_positive"),
        's': (self.update_desired_velocity, [1, -lin_vel], "linear_y_negative"),
        'x': (self.update_desired_velocity, [1, 0], "linear_y_stop"),
        'e': (self.update_desired_velocity, [2, lin_vel], "linear_z_positive"),
        'd': (self.update_desired_velocity, [2, -lin_vel], "linear_z_negative"),
        'c': (self.update_desired_velocity, [2, 0], "linear_z_stop"),
        'r': (self.update_desired_velocity, [3, ang_vel], "angular_x_positive"),
        'f': (self.update_desired_velocity, [3, -ang_vel], "angular_x_negative"),
        'v': (self.update_desired_velocity, [3, 0], "angular_x_stop"),
        't': (self.update_desired_velocity, [4, ang_vel], "angular_y_positive"),
        'g': (self.update_desired_velocity, [4, -ang_vel], "angular_y_negative"),
        'b': (self.update_desired_velocity, [4, 0], "angular_y_stop"),
        'y': (self.update_desired_velocity, [5, ang_vel], "angular_z_positive"),
        'h': (self.update_desired_velocity, [5, -ang_vel], "angular_z_negative"),
        'n': (self.update_desired_velocity, [5, 0], "angular_z_stop"),
    }

    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to call self.update_desired_velocity
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

  def odom_callback(self, data):
    link_frame_current_cartesian_velocity = np.array([data.twist.twist.linear.x,
                                                    data.twist.twist.linear.y,
                                                    data.twist.twist.linear.z,
                                                    data.twist.twist.angular.x,
                                                    data.twist.twist.angular.y,
                                                    data.twist.twist.angular.z])

    # Transform from link to base frame
    quat = self.kin.forward_position_kinematics()[3:]
    rotation = scipy.from_quat(quat)
    base_frame_current_cartesian_velocity = np.hstack([
      rotation.apply(link_frame_current_cartesian_velocity[0:3], inverse=False),
      rotation.apply(link_frame_current_cartesian_velocity[3:], inverse=False)])   
    
    # Update buffer
    current_time =  time.time()
    log_data = np.hstack([time.time(), 
                          self.base_frame_desired_cartesian_velocity,
                          base_frame_current_cartesian_velocity])
    #data = asarray([[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]])
    # save to csv file
    np.savetxt('data.csv', log_data, delimiter=',')
    #self.log_file.write(log_data)

if __name__ == "__main__":
  rospy.init_node("baxter_velocity_teleop")
  keyboard_teleop = KeyboardTeleop()    
  keyboard_teleop.log_file.close()