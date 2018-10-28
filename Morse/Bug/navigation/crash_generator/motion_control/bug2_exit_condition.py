from crash_generator import configuration
from crash_generator.robot.robot import Robot
from crash_generator.trilateration.planar_geometry import Line2D, Point2D


class Bug2ExitCondition(object):
    def __init__(self, robot: Robot, goal: Point2D):
        self._robot = robot
        self._goal = goal
        self._hit_point = None
        self._line_to_goal = None
        self._has_left_hit_point = False

    def evaluate(self):
        if self._hit_point is None:
            self.set_hit_point()
        if (not self._has_left_hit_point) and (not self._is_on_hit_point()):
            self._has_left_hit_point = True
        if self._has_left_hit_point and self._is_on_hit_point():
            self._robot.stop()
            raise Exception('Kein Pfad zu dem Ziel')

        exit_condition = (self._is_on_the_line() and self._has_left_hit_point) or \
                             self._robot.position.distance_from(self._goal) < self._robot.SENSORS_RANGE
        if exit_condition:
            self._hit_point = None
        return exit_condition

    def set_hit_point(self):
        self._has_left_hit_point = False
        self._hit_point = self._robot.position
        self._line_to_goal = Line2D.from_two_points(self._hit_point, self._goal)

    def _is_on_the_line(self):
        return self._line_to_goal.contains_point(self._robot.position, configuration.BUG_2_LINE_TOLERANCE) and \
               self._robot.position.distance_from(self._goal) < self._hit_point.distance_from(self._goal)

    def _is_on_hit_point(self):
        return self._hit_point.is_close_to(self._robot.position, configuration.BUG_2_INITIAL_POSITION_TOLERANCE)


class Bug2WithNoTargetExitCondition(Bug2ExitCondition):

    def evaluate(self):
        return super(Bug2WithNoTargetExitCondition, self).evaluate() or \
               self._robot.distance_to_target <= configuration.PROXIMITY_RANGE
