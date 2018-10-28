from crash_generator import configuration
from crash_generator.robot.robot import Robot


class BoundaryFollower(object):
    """Generic algorithm for boundary following with boolean exit condition"""

    def __init__(self, robot: Robot, exit_condition):
        self._robot = robot
        self._exit_condition = exit_condition

    def follow_till_exit(self):
        """ Makes the robot follow the boundary of the obstacle while a certain condition is met """
        self._robot.stop()

        while not self._exit_condition.evaluate():

            if self._robot.free_space_ahead < self._robot.SENSORS_RANGE:
                self._robot.set_velocity(
                    0,
                    -configuration.BOUNDARY_FOLLOWING_ANGULAR_VELOCITY)
                while self._robot.free_space_ahead < self._robot.SENSORS_RANGE:
                    pass

            elif self._robot.free_space_left < configuration.BOUNDARY_FOLLOWING_MIN_FREE_SPACE:
                self._robot.set_velocity(
                    configuration.BOUNDARY_FOLLOWING_LINEAR_SPEED*0.33,
                    -configuration.BOUNDARY_FOLLOWING_ANGULAR_VELOCITY*0.5)
                while self._robot.free_space_left < configuration.BOUNDARY_FOLLOWING_MIN_FREE_SPACE:
                    pass

            elif self._robot.free_space_left > configuration.BOUNDARY_FOLLOWING_MAX_FREE_SPACE:
                self._robot.set_velocity(
                    configuration.BOUNDARY_FOLLOWING_LINEAR_SPEED*0.33,
                    configuration.BOUNDARY_FOLLOWING_ANGULAR_VELOCITY*0.5)
                while self._robot.free_space_left > configuration.BOUNDARY_FOLLOWING_MAX_FREE_SPACE:
                    pass

            self._robot.set_velocity(configuration.BOUNDARY_FOLLOWING_LINEAR_SPEED, 0).sleep(configuration.SAMPLE_TIME)


