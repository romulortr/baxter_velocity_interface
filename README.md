# EXPERIMENTAL - DO NOT USE IT IF YOU DO NOT KNOW WHAT YOU ARE DOING
This python package implements a velocity interface to control and estimate linear/angular velocity of baxter' cameras.

# Instructions 
To run the interface, use the following node:
```sh
rosrun baxter_velocity_interface velocity_to_position_node.py
```
To run the keyboard teleop (velocity commands in the base-frame):
```sh
rosrun baxter_velocity_interface keyboard_teleop.py
```
