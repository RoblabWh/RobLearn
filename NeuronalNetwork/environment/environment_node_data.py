#!/usr/bin/env python3

import math
import random
import enum

from .environment_node import Node


class Mode(enum.Enum):
    ALL_COMBINATION = 0
    ALL_RANDOM = 1
    CHECKPOINT = 2


class NodeData:

    __data = []
    __indices_start = []
    __indices_end = []
    __indices_checkpoint = []
    __current_start = None
    __current_start_index = 0
    __current_end = None
    __current_end_index = 0

    __mode = Mode.ALL_COMBINATION

    def read_node_file(self, filename="") -> bool:
        file = open(filename, 'r')

        for line in file:
            if line[0] != '#':
                line = line.split(' ')

                if len(line) == 8:
                    x = float(line[0])
                    y = float(line[1])
                    r = float(line[2])
                    is_start = line[3] == "True"
                    is_end = line[4] == "True"
                    number = int(line[5])
                    angle_start = math.radians(float(line[6]))
                    angle_end = math.radians(float(line[7]))

                    if number < 0:
                        print("Warn: Negative id in node -> set id to 0.")
                        number = 0

                    self.__data.append(Node(x, y, r, is_start, is_end, number, angle_start, angle_end))
                else:
                    print("Warn: Invalid argument length in line -> ignore node", len(line))

        # Fill indices
        node_dictionary = dict()

        for i in range(len(self.__data)):
            node = self.__data[i]

            if node.is_start():
                self.__indices_start.append(i)
            if node.is_end():
                self.__indices_end.append(i)

            if node.id() in node_dictionary:
                node_list = node_dictionary[node.id()]
                node_list.append(i)
                node_dictionary[node.id()] = node_list
            else:
                node_dictionary[node.id()] = [i]

        # Fill the dictionary_node into indicies checkpoint
        for id, node_list in sorted(node_dictionary.items()):
            self.__indices_checkpoint.append(node_list)

        # Check if there are enough start and end node
        check_successful = True

        if len(self.__indices_start) == 0:
            print("Error: No start node!")
            check_successful = False

        if len(self.__indices_end) == 0:
            print("Error: No end node!")
            check_successful = False

        if len(self.__indices_start) == 1 and len(self.__indices_end) == 1 and self.__indices_start[0] == self.__indices_end[0]:
            print("Error: Only one node which is start and end node!")
            check_successful = False

        # Set the index to len of the list that it begin with node 1 by start
        self.__current_start_index = len(self.__indices_start)
        self.__current_end_index = len(self.__indices_end)

        tmp = self.__data[self.__indices_start[0]]

        print(tmp.x(), tmp.y())

        return check_successful

    def set_mode(self, mode: Mode):
        self.__mode = mode

    def __select_start_next(self):
        self.__current_start_index += 1

        if not self.__current_start_index < len(self.__indices_start):
            self.__current_start_index = 0

        self.__current_start = self.__data[self.__indices_start[self.__current_start_index]]

    def __select_end_next(self):
        self.__current_end_index += 1

        if not self.__current_end_index < len(self.__indices_end):
            self.__current_end_index = 0

        self.__current_end = self.__data[self.__indices_end[self.__current_end_index]]

    def __select_start_random(self):
        if len(self.__indices_start) < 2:
            self.__current_start = self.__indices_start[0]
        else:
            selected_start_index = self.__current_start_index

            while selected_start_index == self.__current_start_index:
                selected_start_index = random.randint(0, len(self.__indices_start))

            self.__current_start_index = selected_start_index
            self.__current_start = self.__data[self.__indices_start[self.__current_start_index]]

    def __select_end_random(self):
        if len(self.__indices_end) < 2:
            self.__current_end = self.__indices_end[0]
        else:
            selected_end_index = self.__current_end_index

            while selected_end_index == self.__current_end_index:
                selected_end_index = random.randint(0, len(self.__indices_end))

            self.__current_end_index = selected_end_index
            self.__current_end = self.__data[self.__indices_end[self.__current_end_index]]

    def __solve_conflict_start_end_next(self):
        if self.__current_start.id() == self.__current_end.id():
            if len(self.__indices_end) < 2:
                self.__select_start_next()
            elif len(self.__indices_start) < 2:
                self.__select_end_next()
            else:
                self.__select_start_next()

    def __solve_conflict_start_end_random(self):
        if self.__current_start.id() == self.__current_end.id():
            if len(self.__indices_end) < 2:
                self.__select_start_random()
            elif len(self.__indices_start) < 2:
                self.__select_end_random()
            else:
                self.__select_start_random()

    def new_node_selection(self):
        if self.__mode == Mode.ALL_COMBINATION:
            self.__select_start_next()
            self.__select_end_next()
            self.__solve_conflict_start_end_next()
        elif self.__mode == Mode.ALL_RANDOM:
            self.__select_start_random()
            self.__select_end_random()
            self.__solve_conflict_start_end_random()

    def generate_robot_start_position(self):
        radius = random.uniform(0, self.__current_start.radius())
        angle = random.uniform(-math.pi, math.pi)

        # Minus the radius of the robot + 0.05 cm for safety
        radius -= 0.30
        if radius < 0:
            radius = 0

        x = radius * math.cos(angle) + self.__current_start.x()
        y = radius * math.sin(angle) + self.__current_start.y()

        # orientation bias 10 degree
        orientation_bias = math.radians(5)
        orientation = random.uniform(self.__current_start.angle_start() - orientation_bias, self.__current_start.angle_end() + orientation_bias)

        return x, y, orientation

    def get_node_start(self) -> Node:
        return self.__current_start

    def get_node_end(self) -> Node:
        return self.__current_end
