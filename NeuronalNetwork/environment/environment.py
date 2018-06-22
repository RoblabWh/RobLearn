#!/usr/bin/env python3

from pysim2d import pysim2d
from .environment_fitness import FitnessData

PATH_TO_WORLD = "../Simulation2d/world/"


class Environment:
    __fitness_data = FitnessData()
    __env = pysim2d.pysim2d()
    __cluster_size = 1

    def __init__(self, world_name=""):
        if not self.__fitness_data.init(PATH_TO_WORLD + world_name + ".node"):
            print("Error: Load node file! -> " + world_name + ".node")
            exit(1)
        if not self.__env.init(PATH_TO_WORLD + world_name + ".world"):
            print("Error: Load world file -> " + world_name + ".world")
            exit(1)

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

    def set_cluster_size(self, size):
        self.__cluster_size = size

    def observation_size(self):
        if self.__cluster_size < 2:
            return self.__env.observation_size()
        else:
            return self.__env.observation_min_clustered_size(self.__cluster_size)

    def visualize(self):
        self.__env.visualize()

    def step(self, linear_velocity: float, angular_velocity: float, skip_number: int = 1):
        self.__env.step(linear_velocity, angular_velocity, skip_number)

        reward, done = self.__fitness_data.calculate_reward(self.__env.get_robot_pose_x(),
                                                            self.__env.get_robot_pose_y(),
                                                            self.__env.get_robot_pose_orientation(),
                                                            self.__env.done())

        if self.__cluster_size < 2:
            observation = self.__get_observation()
        else:
            observation = self.__get_observation_min_clustered(self.__cluster_size)

        return observation, reward, done, ""

    def reset(self):
        self.__fitness_data.reset()
        x, y, orientation = self.__fitness_data.get_robot_start()
        self.__env.set_robot_pose(x, y, orientation)
        return self.step(0.0, 0.0)
