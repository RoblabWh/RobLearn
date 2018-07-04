#!/usr/bin/env python3

from pysim2d import pysim2d
from .environment_fitness import FitnessData
from .environment_node_data import Mode

PATH_TO_WORLD = "../Simulation2d/world/"


class Environment:

    def __init__(self, world_name=""):
        self._env = pysim2d.pysim2d()
        self._fitness_data = FitnessData()
        self._cluster_size = 1
        if not self._fitness_data.init(PATH_TO_WORLD + world_name + ".node"):
            print("Error: Load node file! -> " + world_name + ".node")
            exit(1)
        if not self._env.init(PATH_TO_WORLD + world_name + ".world"):
            print("Error: Load world file -> " + world_name + ".world")
            exit(1)

    def _get_observation(self):
        size = self._env.observation_size()
        observation = []

        for i in range(size):
            observation.append(self._env.observation_at(i))

        return observation

    def _get_observation_min_clustered(self, cluster_size: int):
        size = self._env.observation_min_clustered_size(self._cluster_size)
        observation = []

        for i in range(size):
            observation.append(self._env.observation_min_clustered_at(i, self._cluster_size))

        return observation

    def set_cluster_size(self, size):
        self._cluster_size = size

    def observation_size(self):
        if self._cluster_size < 2:
            return self._env.observation_size()
        else:
            return self._env.observation_min_clustered_size(self._cluster_size)

    def visualize(self):
        self._env.visualize()

    def step(self, linear_velocity: float, angular_velocity: float, skip_number: int = 1):
        self._env.step(linear_velocity, angular_velocity, skip_number)

        reward, done = self._fitness_data.calculate_reward(self._env.get_robot_pose_x(),
                                                           self._env.get_robot_pose_y(),
                                                           self._env.get_robot_pose_orientation(),
                                                           self._env.done())

        if self._cluster_size < 2:
            observation = self._get_observation()
        else:
            observation = self._get_observation_min_clustered(self._cluster_size)

        return observation, reward, done, ""

    def reset(self):
        self._fitness_data.reset()
        x, y, orientation = self._fitness_data.get_robot_start()
        self._env.set_robot_pose(x, y, orientation)
        return self.step(0.0, 0.0)

    def set_mode(self, mode):
        self._fitness_data.set_mode(mode)