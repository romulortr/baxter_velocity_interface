
#!/usr/bin/env python
import rospy

import baxter_interface
import baxter_external_devices

from geometry_msgs.msg import Twist


class KeyboardTeleop:
  def __init__(self):
    #self.kin = baxter_kinematics('right') 

    rate = 20 # hz
    
    self.twist_pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=10)
    
    self.base_frame_desired_cartesian_velocity = np.zeros(6)

    rospy.Timer(rospy.Duration(1./rate), self.twist_publisher)    
    print("Controlling joints. Press ? for help, Esc to quit.")
    map_keyboard()

  def twist_publisher(self, event):
    # Transform from link to base frame
    quat = self.kin.forward_position_kinematics()[3:]
    rotation = scipy.from_quat(quat)
    link_frame_desired_cartesian_velocity = np.hstack([
      rotation.apply(self.base_frame_desired_cartesian_velocity[0:3], inverse=True),
      rotation.apply(self.base_frame_desired_cartesian_velocity[3:], inverse=True)])   

    # Write message and send it
    twist_msg = Twist()
    twist_msgs.linear.x = link_frame_desired_cartesian_velocity[0]
    twist_msgs.linear.y = link_frame_desired_cartesian_velocity[1]
    twist_msgs.linear.z = link_frame_desired_cartesian_velocity[2]
    twist_msgs.angular.x = link_frame_desired_cartesian_velocity[3]
    twist_msgs.angular.y = link_frame_desired_cartesian_velocity[4]
    twist_msgs.angular.z = link_frame_desired_cartesian_velocity[5]
    
    self.twist_pub.publish(twist_msg)

  def update_desired_velocity(self, field, value):
    self.base_frame_desired_cartesian_velocity[field] = value

  def map_keyboard(self):
    bindings = {
    #   key: (function, args, description)
        'q': (update_desired_velocity, [0, 0.1], "linear_x_positive"),
        'a': (update_desired_velocity, [0, -0.1], "linear_x_negative"),
        'z': (update_desired_velocity, [0, 0], "linear_x_stop"),
        'w': (update_desired_velocity, [1, 0.1], "linear_y_positive"),
        's': (update_desired_velocity, [1, -0.1], "linear_y_negative"),
        'x': (update_desired_velocity, [1, 0], "linear_y_stop"),
        'e': (update_desired_velocity, [2, 0.1], "linear_z_positive"),
        'd': (update_desired_velocity, [2, -0.1], "linear_z_negative"),
        'c': (update_desired_velocity, [2, 0], "linear_z_stop"),
        'r': (update_desired_velocity, [3, 0.1], "angular_x_positive"),
        'f': (update_desired_velocity, [3, -0.1], "angular_x_negative"),
        'v': (update_desired_velocity, [3, 0], "angular_x_stop"),
        't': (update_desired_velocity, [4, 0.1], "angular_y_positive"),
        'g': (update_desired_velocity, [4, -0.1], "angular_y_negative"),
        'b': (update_desired_velocity, [4, 0], "angular_y_stop"),
        'y': (update_desired_velocity, [5, 0.1], "angular_z_positive"),
        'h': (update_desired_velocity, [5, -0.1], "angular_z_negative"),
        'n': (update_desired_velocity, [5, 0], "angular_z_stop"),
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
                #expand binding to something like "set_j(right, 's0', 0.1)"
                # cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

if __name__ == "__main__":
  rospy.init_node("baxter_velocity_teleop")
  print('Entering')
  keyboard_teleop = KeyboardTeleop()    
