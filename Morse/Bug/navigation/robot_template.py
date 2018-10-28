#! /usr/bin/env python3
"""
Dies Programm kontrolliert der Roboter mit Python.
Es gibt die Möglichkeit alle Sensoren und Middlewares wie ROS hier der Simulation bereitzustellen.
Die Roboterplattform und ihre Sensoren bzw. Aktuatoren können in der Datei atr_with_ir.toml im Hauptverzeichnis zu initialisieren oder ändern.
Atrv kann durch alle bellibige Roboter vom Morse Komponenten Biblithek ersetzt werden.

"""

import sys

try:
    from pymorse import Morse
except ImportError:
    print("Bitte installieren Sie erst Pymorse!")
    sys.exit(1)

with Morse() as simu:

    # Definition of the sensors and the actuator
    pose = simu.robot.pose                  # pose sensor
    ir1 = simu.robot.IR1                    # ir sensor #1
    ir2 = simu.robot.IR2                    # ir sensor #2
    ir3 = simu.robot.IR3                    # ir sensor #3
    ir4 = simu.robot.IR4                    # ir sensor #4
    prox = simu.robot.prox                  # proximity sensor
    motion = simu.robot.motion              # motion speed actuator

    # Get sensor measurements and control the actuator
    pose.get()                              # get from pose sensor
    ir1.get()                               # get from ir sensor #1
    ir2.get()                               # get from ir sensor #2
    ir3.get()                               # get from ir sensor #3
    ir4.get()                               # get from ir sensor #4
    prox.get()                              # get from proximity sensor
    motion.publish({"v": 0, "w": 0})        # set robot linear and angular speed
