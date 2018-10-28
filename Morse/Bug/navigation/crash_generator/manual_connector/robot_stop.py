import sys

from crash_generator.robot.robot import Robot

try:
    from pymorse import Morse
except ImportError:
    print("Bitte installieren Sie erst Pymorse!")
    sys.exit(1)


with Morse() as sim:
    robot = Robot(sim.robot, sim)
    robot.stop()

