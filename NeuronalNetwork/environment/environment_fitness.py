#!/usr/bin/env python3

import math
from .environment_node_data import NodeData, Mode


class FitnessData:

    def __init__(self):
        self._node_data = NodeData()

        self._terminate_at_end = True

        self._robot_x_last = 0.0
        self._robot_y_last = 0.0
        self._robot_orientation_last = 0.0

    def get_end_node(self):
        return self._node_data.get_node_end()

    def init(self, filename="") -> bool:
        return self._node_data.read_node_file(filename)

    def set_mode(self, mode: Mode, terminate_at_end=True):
        self._node_data.set_mode(mode)
        self._terminate_at_end = terminate_at_end

    def _distance_start_to_end(self) -> float:
        return self._distance(
            self._node_data.get_node_start().x(),
            self._node_data.get_node_start().y(),
            self._node_data.get_node_end().x(),
            self._node_data.get_node_end().y())

    def _distance_robot_to_end(self, robot_x: float, robot_y: float) -> float:
        return self._distance(
            robot_x,
            robot_y,
            self._node_data.get_node_end().x(),
            self._node_data.get_node_end().y())

    def _distance_between_last_step(self, robot_x: float, robot_y: float) -> float:
        return self._distance(
            robot_x,
            robot_y,
            self._robot_x_last,
            self._robot_y_last)

    def _orientation_robot_to_end(self, robot_x: float, robot_y: float) -> float:
        x = self._node_data.get_node_end().x() - robot_x
        y = self._node_data.get_node_end().y() - robot_y
        return math.atan2(y, x)

    def angle_from_robot_to_end(self) -> float:
        return self._difference_two_angles(self._robot_orientation_last, self._orientation_robot_to_end(self._robot_x_last, self._robot_y_last))

    @staticmethod
    def _difference_two_angles(angle1: float, angle2: float) -> float:
        diff = (angle1 - angle2) % (math.pi * 2)
        if diff >= math.pi:
            diff -= math.pi * 2
        return diff

    @staticmethod
    def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
        x = x1 - x2
        y = y1 - y2
        return math.sqrt(x*x + y*y)

    def reset(self):
        self._node_data.new_node_selection()

    def calculate_reward(self, robot_x: float, robot_y: float, robot_orientation: float, env_done: bool):
        done = False

        #distance_start_to_end = self.__distance_start_to_end()
        distance_robot_to_end = self._distance_robot_to_end(robot_x, robot_y)
        #distance_between_last_step = self.__distance_between_last_step(robot_x, robot_y)

        #reward = distance_between_last_step + (1 - distance_robot_to_end / distance_start_to_end) + distance_between_last_step

        reward = 0
        #reward = (math.pi - math.fabs(self._difference_two_angles(robot_orientation, self._orientation_robot_to_end(robot_x, robot_y)))) / math.pi
        #reward += max((5 - self._distance_robot_to_end(robot_x, robot_y)) / 5, 0)
        if env_done:
            reward = -100 #- distance_robot_to_end / distance_start_to_end * 100
            done = True
        elif distance_robot_to_end < self._node_data.get_node_end().radius():
            reward = 100
            done = self._handle_terminate_at_end()
        else:
            reward = 1
            # reward = -1

        self._robot_x_last = robot_x
        self._robot_y_last = robot_y
        self._robot_orientation_last = robot_orientation

        return reward, done

    def get_robot_start(self):
        return self._node_data.generate_robot_start_position()

    def _handle_terminate_at_end(self):
        if not self._terminate_at_end:
            self._node_data.new_end_node()

        return self._terminate_at_end
