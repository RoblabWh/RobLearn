#!/usr/bin/env python3


class Node:

    def __init__(self, x: float, y: float, r: float, is_start: bool, is_end: bool, number: int, angle_start: float, angle_end: float):
        self._x = x
        self._y = y
        self._r = r
        self._is_start = is_start
        self._is_end = is_end
        self._id = number
        self._angle_start = angle_start
        self._angle_end = angle_end

    def x(self) -> float:
        return self._x

    def y(self) -> float:
        return self._y

    def radius(self) -> float:
        return self._r

    def is_start(self) -> bool:
        return self._is_start

    def is_end(self) -> bool:
        return self._is_end

    def id(self) -> int:
        return self._id

    def angle_start(self) -> float:
        return self._angle_start

    def angle_end(self) -> float:
        return self._angle_end
