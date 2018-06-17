#!/usr/bin/env python3

class Node:
    __x = 0.0
    __y = 0.0
    __r = 0.0
    __is_start = False
    __is_end = False
    __id = 0
    __angle_start = 0.0
    __angle_end = 0.0

    def __init__(self, x: float, y: float, r: float, is_start: bool, is_end: bool, number: int, angle_start: float, angle_end: float):
        self.__x = x
        self.__y = y
        self.__r = r
        self.__is_start = is_start
        self.__is_end = is_end
        self.__id = number
        self.__angle_start = angle_start
        self.__angle_end = angle_end

    def x(self) -> float:
        return self.__x

    def y(self) -> float:
        return self.__y

    def radius(self) -> float:
        return self.__r

    def is_start(self) -> bool:
        return self.__is_start

    def is_end(self) -> bool:
        return self.__is_end

    def id(self) -> int:
        return self.__id

    def angle_start(self) -> float:
        return self.__angle_start

    def angle_end(self) -> float:
        return self.__angle_end
