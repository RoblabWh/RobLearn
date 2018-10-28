#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <turtlebot_sim> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from turtlebot_sim.builder.robots import Turtlebot

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> turtlebot_sim' can help you to build custom robots.
robot = Turtlebot()

#robot = ATRV()

## Append Pioneer robot to the scene
#robot = Pioneer3DX()
#robot.rotate(z=0.73)
#robot.translate(x=-2.0, z=0.2)
#robot.unparent_wheels()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(0.0, 0.0, 0.0)

# Motion controller is added in turtlebot.py
# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> turtlebot_sim' can help you with the creation of a custom
# actuator.
#motion = MotionVW()
#robot.append(motion)

# Add a keyboard controller to move the robot with arrow keys.
#keyboard = Keyboard()
#robot.append(keyboard)

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> turtlebot_sim' can help you with the creation of a custom
# sensor.
#pose = Pose()
#robot.append(pose)

#odometry = Odometry()
#robot.append(odometry)
#odometry.add_interface('ros', topic="/odom")


## creates a new instance of the sensor
#kinect = Kinect()
## place your component at the correct location
##kinect.translate(0,0,0)
#kinect.rotate(0,0,0)
#robot.append(kinect)

# Append a sick laser
#sick = Sick() # range: 30m, field: 180deg, 180 sample points
#sick.translate(0,0,0.9)
#sick.rotate(0,0,0)
#robot.append(sick)

# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html 
# the other available interfaces (like ROS, YARP...)
robot.add_default_interface('socket')
#robot.add_default_interface('ros')


# set 'fastmode' to True to switch to wireframe mode
env = Environment('laas/grande_salle', fastmode = True)
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.05, 0, 0.78])

