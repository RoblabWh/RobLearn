from crash_generator import configuration
from crash_generator.motion_control.boundary_following import BoundaryFollower
from crash_generator.robot.robot import Robot


class ConstantMotion(object):

    def __init__(self, robot: Robot, linear_vel, angular_vel, exit_condition):
        self._robot = robot
        self._linear_vel = linear_vel
        self._angular_vel = angular_vel
        self._exit_condition = exit_condition

    def go_until_exit(self):
        while not (self._exit_condition.evaluate() or self._robot.risk_of_collision):
            self._robot.set_velocity(self._linear_vel, self._angular_vel)
            self._robot.sleep(configuration.SAMPLE_TIME)
        self._robot.stop()

