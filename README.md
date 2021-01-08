This python package implements a velocity interface to control and estimate linear/angular velocity of baxter' cameras.
### Requirements
1. Baxter SDK (https://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
2. Baxter PyKDL (https://github.com/RethinkRobotics/baxter_pykdl)

### Instructions 
To run the interface using baxter_gazebo:
```sh
rosrun baxter_velocity_interface gazebo.py
```
To run the interface using baxter (real robot):
```sh
rosrun baxter_velocity_interface baxter.py
```


To run the keyboard teleop (velocity commands in the base-frame):
```sh
rosrun baxter_velocity_interface keyboard_teleop.py
```
