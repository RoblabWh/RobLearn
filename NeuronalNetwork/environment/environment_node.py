#!/usr/bin/env python3


class Node:
    """
    The class represent a node. Defined by position, radius, id and start/end node.
    """
    def __init__(self, x: float, y: float, r: float, is_start: bool, is_end: bool, number: int, angle_start: float, angle_end: float):
        """
        Constructor of the node class.
        :param x: X position.
        :param y: Y position.
        :param r: Radius.
        :param is_start: Is a start node.
        :param is_end: Is a end node.
        :param number: Id number.
        :param angle_start: Angle start.
        :param angle_end: Angle end.
        """
        self._x = x
        self._y = y
        self._r = r
        self._is_start = is_start
        self._is_end = is_end
        self._id = number
        self._angle_start = angle_start
        self._angle_end = angle_end

    def x(self) -> float:
        """
        Get the x position.
        :return: X position.
        """
        return self._x

    def y(self) -> float:
        """
        Get the y position.
        :return: Y position.
        """
        return self._y

    def radius(self) -> float:
        """
        Get the radius.
        :return: Radius.
        """
        return self._r

    def is_start(self) -> bool:
        """
        True if the node is a start node.
        :return: True when start node.
        """
        return self._is_start

    def is_end(self) -> bool:
        """
        True if the node is a end node.
        :return: True when end node.
        """
        return self._is_end

    def id(self) -> int:
        """
        Get the node id.
        :return: Node id.
        """
        return self._id

    def angle_start(self) -> float:
        """
        Get the start angle of the node.
        :return: Start angle.
        """
        return self._angle_start

    def angle_end(self) -> float:
        """
        Get the end angle of the node.
        :return: End angle.
        """
        return self._angle_end
