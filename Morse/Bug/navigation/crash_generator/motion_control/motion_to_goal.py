import math
from threading import Timer

from crash_generator import configuration
from crash_generator.robot.robot import Robot


class MotionToGoal(object):
    def __init__(self, robot: Robot, goal,
                 motion_velocity=configuration.MOTION_TO_GOAL_LINEAR_SPEED,
                 rot_velocity=configuration.MOTION_TO_GOAL_ANGULAR_VELOCITY):
        self.robot = robot
        self.goal = goal
        self.motion_velocity = motion_velocity
        self.rotation_velocity = rot_velocity
        self._route_recompute = True

    def go_until_target_or_obstacle(self):
        self.robot.stop()

        while self.robot.free_space_ahead >= self.robot.SENSORS_RANGE and \
                        self.robot.position.distance_from(self.goal) >= self.robot.SENSORS_RANGE:
            if self._route_recompute:
                self.robot.stop()
                self._rotate_to_goal()
                self._route_recompute = False
                Timer(configuration.MOTION_TO_GOAL_TRAJECTORY_CORRECTION_INTERVAL,
                      self._set_recompute_flag).start()
                if self.robot.free_space_ahead >= self.robot.SENSORS_RANGE:
                    self.robot.set_velocity(self.motion_velocity, 0)

    def _rotate_to_goal(self):
        vector_to_goal = self.goal - self.robot.position
        goal_orientation = math.atan2(vector_to_goal.y, vector_to_goal.x)
        self.robot.rotate_to(goal_orientation, self.rotation_velocity)

    def _set_recompute_flag(self):
        self._route_recompute = True
