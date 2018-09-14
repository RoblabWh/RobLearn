#!/usr/bin/env python3

import math
import random
import enum
from datetime import datetime

from .environment_node import Node


class Mode(enum.Enum):
    """
    Mode for the organisition of the node selection.
    """
    ALL_COMBINATION = 0
    ALL_RANDOM = 1
    PAIR_ALL = 2
    PAIR_RANDOM = 3
    CHECKPOINT = 4


class NodeData:
    """
    This class organized the node. Depending on the mode the start and target node are selected.
    """
    def __init__(self):
        self._data = []
        self._indices_start = []
        self._indices_end = []
        self._indices_pair_start = []
        self._indices_pair_end = []
        self._indices_checkpoint = []
        self._current_start = None
        self._current_start_index = 0
        self._current_end = None
        self._current_end_index = 0
        self._current_pair_index = 0

    _mode = Mode.ALL_COMBINATION

    def read_node_file(self, filename="") -> bool:
        """
        Read the node file to load the node.
        :param filename: Node filename string.
        :return:
        """
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

                    self._data.append(Node(x, y, r, is_start, is_end, number, angle_start, angle_end))
                else:
                    print("Warn: Invalid argument length in line -> ignore node", len(line))

        # Fill indices
        node_dictionary = dict()

        for i in range(len(self._data)):
            node = self._data[i]

            if node.is_start():
                self._indices_start.append(i)
            if node.is_end():
                self._indices_end.append(i)

            if node.id() in node_dictionary:
                node_list = node_dictionary[node.id()]
                node_list.append(i)
                node_dictionary[node.id()] = node_list
            else:
                node_dictionary[node.id()] = [i]

        # Fill the dictionary_node into indicies checkpoint
        for id, node_list in sorted(node_dictionary.items()):
            self._indices_checkpoint.append(node_list)

            node_list_start = []
            node_list_end = []

            # Fill the pair list with start and end node indices
            for index in node_list:
                node = self._data[index]

                if node.is_start():
                    node_list_start.append(index)

                if node.is_end():
                    node_list_end.append(index)

            # Check if there are enough start and end node
            check_pair = True

            if len(node_list_start) == 0:
                print("Warn: No start node in pair -> skip", id)
                check_pair = False

            if len(node_list_end) == 0:
                print("Warn: No end node in pair -> skip", id)
                check_pair = False

            if len(node_list_start) == 1 and len(node_list_end) == 1 and node_list_start[0] == node_list_end[0]:
                print("Warn: Only one node which is start and end node in pair -> skip!", id)
                check_pair = False

            if check_pair:
                self._indices_pair_start.append(node_list_start)
                self._indices_pair_end.append(node_list_end)

        # Check if there are enough start and end node
        check_successful = True

        if len(self._indices_start) == 0:
            print("Error: No start node!")
            check_successful = False

        if len(self._indices_end) == 0:
            print("Error: No end node!")
            check_successful = False

        if len(self._indices_start) == 1 and len(self._indices_end) == 1 and self._indices_start[0] == self._indices_end[0]:
            print("Error: Only one node which is start and end node!")
            check_successful = False

        # Set the index to len of the list that it begin with node 1 by start
        self._current_start_index = len(self._indices_start)
        self._current_end_index = len(self._indices_end)

        tmp = self._data[self._indices_start[0]]

        return check_successful

    def set_mode(self, mode: Mode):
        """
        Set the mode for the node selection.
        :param mode: Simulation mode.
        :return:
        """
        random.seed(datetime.now())
        self._mode = mode

    def get_mode(self):
        """
        Get the mode.
        :return: Mode.
        """
        return self._mode

    def _select_start_next(self, indices_start):
        """
        Select the next start node from the start node collection.
        :param indices_start: Current start node id.
        :return:
        """
        self._current_start_index += 1

        if not self._current_start_index < len(indices_start):
            self._current_start_index = 0

        self._current_start = self._data[indices_start[self._current_start_index]]

    def _select_end_next(self, indices_end):
        """
        Select the next end node from the end node collection.
        :param indices_end: Currecnt end node id.
        :return:
        """
        self._current_end_index += 1

        if not self._current_end_index < len(indices_end):
            self._current_end_index = 0

        self._current_end = self._data[indices_end[self._current_end_index]]

    def _select_start_random(self, indices_start):
        """
        Select a random start node from the start node collection.
        :param indices_start: Current start node id.
        :return:
        """
        if len(indices_start) < 2:
            self._current_start = indices_start[0]
        else:
            selected_start_index = self._current_start_index

            while selected_start_index == self._current_start_index:
                selected_start_index = random.randint(0, len(indices_start) - 1)

            self._current_start_index = selected_start_index
            self._current_start = self._data[indices_start[self._current_start_index]]

    def _select_end_random(self, indices_end):
        """
        Select a random end node from the end node collection.
        :param indices_end: Current end node id.
        :return:
        """
        if len(indices_end) < 2:
            self._current_end = indices_end[0]
        else:
            selected_end_index = self._current_end_index

            while selected_end_index == self._current_end_index:
                selected_end_index = random.randint(0, len(indices_end) - 1)

            self._current_end_index = selected_end_index
            self._current_end = self._data[indices_end[self._current_end_index]]

    def _solve_conflict_start_end_next(self, indices_start, indices_end):
        """
        Solve the conflict when the start and end node is the same.
        :param indices_start: Indices list of the start node.
        :param indices_end: Indices list of the end node.
        :return:
        """
        if indices_start[self._current_start_index] == indices_end[self._current_end_index]:
            if len(indices_end) < 2:
                self._select_start_next(indices_start)
            elif len(indices_start) < 2:
                self._select_end_next(indices_end)
            else:
                self._select_start_next(indices_start)

    def _solve_conflict_start_end_random(self, indices_start, indices_end):
        """
        Solve the conflict when the start and end node is the same by a random selection.
        :param indices_start: Indices list of the start node.
        :param indices_end: Indices list of the end node.
        :return:
        """
        if self._indices_start[self._current_start_index] == self._indices_end[self._current_end_index]:
            if len(indices_end) < 2:
                self._select_start_random(indices_start)
            elif len(indices_start) < 2:
                self._select_end_random(indices_end)
            else:
                self._select_start_random(indices_start)

    def _select_pair_next(self):
        """
        Select the next pair.
        :return:
        """
        self._current_pair_index += 1

        if not self._current_pair_index < len(self._indices_pair_start):
            self._current_pair_index = 0

        self._select_start_random(self._indices_pair_start[self._current_pair_index])
        self._select_end_random(self._indices_pair_end[self._current_pair_index])
        self._solve_conflict_start_end_random(self._indices_pair_start[self._current_pair_index], self._indices_pair_end[self._current_pair_index])

    def _select_pair_random(self):
        """
        Select a random pair.
        :return:
        """
        if len(self._indices_pair_start) < 2:
            self._current_pair_index = 0
        else:
            selected_pair_index = self._current_pair_index

            while selected_pair_index == self._current_pair_index:
                selected_pair_index = random.randint(0, len(self._indices_pair_start) - 1)

            self._current_pair_index = selected_pair_index

        self._select_start_random(self._indices_pair_start[self._current_pair_index])
        self._select_end_random(self._indices_pair_end[self._current_pair_index])
        self._solve_conflict_start_end_random(self._indices_pair_start[self._current_pair_index], self._indices_pair_end[self._current_pair_index])

    def new_node_selection(self):
        """
        Select a new start and end node.
        :return:
        """
        if self._mode == Mode.ALL_COMBINATION:
            self._select_start_next(self._indices_start)
            self._select_end_next(self._indices_end)
            self._solve_conflict_start_end_next(self._indices_start, self._indices_end)
        elif self._mode == Mode.ALL_RANDOM:
            self._select_start_random(self._indices_start)
            self._select_end_random(self._indices_end)
            self._solve_conflict_start_end_random(self._indices_start, self._indices_end)
        elif self._mode == Mode.PAIR_ALL:
            self._select_pair_next()
        elif self._mode == Mode.PAIR_RANDOM:
            self._select_pair_random()

    def generate_robot_start_position(self):
        """
        Generate the start position for the robot of a node with bias.
        :return:
        """
        radius = random.uniform(0, self._current_start.radius())
        angle = random.uniform(-math.pi, math.pi)

        # Minus the radius of the robot + 2.5 cm for safety
        radius -= 0.20
        if radius < 0:
            radius = 0

        x = radius * math.cos(angle) + self._current_start.x()
        y = radius * math.sin(angle) + self._current_start.y()

        # orientation bias 10 degree
        orientation_bias = math.radians(5)
        orientation = random.uniform(self._current_start.angle_start() - orientation_bias, self._current_start.angle_end() + orientation_bias)

        return x, y, orientation

    def get_node_start(self) -> Node:
        """
        Get the current start node.
        :return:
        """
        return self._current_start

    def get_node_end(self) -> Node:
        """
        Get the current target node.
        :return:
        """
        random.seed(datetime.now())
        return self._current_end

    def new_end_node(self):
        """
        Select a new end.
        :return:
        """

        if self._mode == Mode.PAIR_ALL or self._mode == Mode.PAIR_RANDOM:
            self._select_end_random(self._indices_pair_end[self._current_pair_index])
        else:
            self._select_end_random(self._indices_end)
