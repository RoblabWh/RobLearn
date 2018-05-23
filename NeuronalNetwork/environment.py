#!/usr/bin/env python3

from pysim2d import pysim2d
import math
import numpy as np

class Environment:
    __env = pysim2d.pysim2d()
    __target_x = 0
    __target_y = 0
    __start_x = 0
    __start_y = 0
    __start_orientation = 0
    __last_x = 0
    __last_y = 0
    __last_orientation = 0
    __target_radius = 1
    __cluster_size = 1

    def __init__(self):
        self.__env.init("")

    def __distance(self, x1:float, x2:float, y1:float, y2:float):
        x = x1 - x2
        y = y1 - y2
        return math.sqrt(x*x + y*y)

    def set_cluster_size(self, size):
        self.__cluster_size = size

    def observation_size(self):
        if self.__cluster_size < 2:
            return self.__env.observation_size()
        else:
            return self.__env.observation_min_clustered_size(self.__cluster_size)

    def __get_observation(self):
        size = self.__env.observation_size()
        observation = []

        for i in range(size):
            observation.append(self.__env.observation_at(i))

        return observation

    def __get_observation_min_clustered(self, cluster_size: int):
        size = self.__env.observation_min_clustered_size(self.__cluster_size)
        observation = []
        
        for i in range(size):
            observation.append(self.__env.observation_min_clustered_at(i, self.__cluster_size))

        return observation

    def visualize(self):
        self.__env.visualize()

    def __calculate_reward(self):
        done = False

        robot_x = self.__env.get_robot_pose_x()
        robot_y = self.__env.get_robot_pose_y()
        robot_orientation = self.__env.get_robot_pose_orientation()

        distance_between_start_and_target = self.__distance(self.__start_x, self.__target_x, self.__start_y, self.__target_y)
        distance_to_target = self.__distance(self.__target_x, self.__last_x, self.__target_y, self.__last_y);
        distance_between_last_step = distance_to_target - self.__distance(self.__target_x, robot_x, self.__target_y, robot_y)

        reward = (1 - distance_to_target / distance_between_start_and_target) + distance_between_last_step

        if self.__env.done():
            reward = - distance_to_target / distance_between_start_and_target * 100
            done = True
        elif distance_to_target < self.__target_radius:
            reward = 100 + reward
            done = True

        self.__last_x = robot_x
        self.__last_y = robot_y
        self.__last_orientation = robot_orientation

        return reward, done

    def set_target(self, x:float, y:float):
        self.__target_x = x
        self.__target_y = y
        
    def set_start(self, x:float, y:float, orientation:float):
        self.__start_x = x
        self.__start_y = y
        self.__start_orientation = orientation
        self.__env.set_robot_pose(x,y,orientation)

    def step(self,linear_velocity:float, angular_velocity:float, skip_number:int = 1):
        self.__env.step(linear_velocity, angular_velocity,skip_number)

        reward, done = self.__calculate_reward()
        observation = []

        if self.__cluster_size < 2:
            observation = self.__get_observation()
        else:
            observation = self.__get_observation_min_clustered(self.__cluster_size)

        return observation, reward, done, ""
        
        