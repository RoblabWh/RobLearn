from pymorse import Morse
from crash_generator.robot.robot import Robot


def get_robot():
    return Robot(Morse().robot, Morse())
