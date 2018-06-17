#!/usr/bin/env python3

import math
from environment_node_data import NodeData, Mode


class FitnessData:

    __node_data = NodeData()

    __terminate_end = True

    __robot_x_last = 0.0
    __robot_y_last = 0.0
    __robot_orientation_last = 0.0

    def init(self, filename="") -> bool:
        return self.__node_data.read_node_file(filename)

    def set_mode(self, mode: Mode):
        self.__node_data.set_mode(mode)

    def __distance_start_to_end(self) -> float:
        return self.__distance(
            self.__node_data.get_node_start().x(),
            self.__node_data.get_node_start().y(),
            self.__node_data.get_node_end().x(),
            self.__node_data.get_node_end().y())

    def __distance_robot_to_end(self, robot_x: float, robot_y: float) -> float:
        return self.__distance(
            robot_x,
            robot_y,
            self.__node_data.get_node_end().x(),
            self.__node_data.get_node_end().y())

    def __distance_between_last_step(self, robot_x: float, robot_y: float) -> float:
        return self.__distance(
            robot_x,
            robot_y,
            self.__robot_x_last,
            self.__robot_y_last)

    def __orientation_robot_to_end(self, robot_x: float, robot_y: float):
        x = self.__node_data.get_node_end().x() - robot_x
        y = self.__node_data.get_node_end().y() - robot_y
        return math.atan2(y, x)

    @staticmethod
    def __difference_two_angles(angle1: float, angle2: float) -> float:
        diff = (angle1 - angle2) % (math.pi * 2)
        if diff >= math.pi:
            diff -= math.pi * 2
        return diff

    @staticmethod
    def __distance(x1: float, y1: float, x2: float, y2: float) -> float:
        x = x1 - x2
        y = y1 - y2
        return math.sqrt(x*x + y*y)

    def reset(self):
        self.__node_data.new_node_selection()

    def calculate_reward(self, robot_x: float, robot_y: float, robot_orientation: float, env_done: bool):
        done = False

        #distance_start_to_end = self.__distance_start_to_end()
        distance_robot_to_end = self.__distance_robot_to_end(robot_x, robot_y)
        #distance_between_last_step = self.__distance_between_last_step(robot_x, robot_y)

        #reward = distance_between_last_step + (1 - distance_robot_to_end / distance_start_to_end) + distance_between_last_step

        reward = (math.pi - math.fabs(self.__difference_two_angles(robot_orientation, self.__orientation_robot_to_end(robot_x, robot_y)))) / math.pi
        if env_done:
            reward = -100 #- distance_robot_to_end / distance_start_to_end * 100
            done = True
        elif distance_robot_to_end < self.__node_data.get_node_end().radius():
            reward = 100
            done = self.__terminate_end

        self.__robot_x_last = robot_x
        self.__robot_y_last = robot_y
        self.__robot_orientation_last = robot_orientation

        return reward, done

    def get_robot_start(self):
        return self.__node_data.generate_robot_start_position()


