#!/usr/bin/env python3

import math

from pysim2d import pysim2d
from .environment_fitness import FitnessData
from .environment_node import Node


PATH_TO_WORLD = "../Simulation2d/world/"


class Environment:

    def __init__(self, world_name=""):
        self._env = pysim2d.pysim2d()
        self._fitness_data = FitnessData()
        self._cluster_size = 1
        self._observation_rotation_size = 64
        self._observation_rotation_use = False

        if not self._fitness_data.init(PATH_TO_WORLD + world_name + ".node"):
            print("Error: Load node file! -> " + world_name + ".node")
            exit(1)
        if not self._env.init(PATH_TO_WORLD + world_name + ".world"):
            print("Error: Load world file -> " + world_name + ".world")
            exit(1)

    def set_observation_rotation_size(self, size):
        if size < 8:
            print("Warn: Observation rotation size is to low -> set to 8!")

            self._observation_rotation_size = 8
        else:
            self._observation_rotation_size = size

    def use_observation_rotation_size(self, use=True):
        self._observation_rotation_use = use

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
            size = self._env.observation_size()
        else:
            size = self._env.observation_min_clustered_size(self._cluster_size)

        if self._observation_rotation_use:
            size += self._observation_rotation_size

        return size

    def visualize(self):
        end_node = self._fitness_data.get_end_node()
        self._env.visualize(end_node.x(), end_node.y(), end_node.radius())

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

        if self._observation_rotation_use:
            not_set = True

            angle_target = self._fitness_data.angle_from_robot_to_end()
            angle_step_size = 2 * math.pi / self._observation_rotation_size
            angle_sum = - math.pi + angle_step_size

            for i in range(self._observation_rotation_size):
                if not_set and angle_target < angle_sum:
                    observation.append(1.0)
                    not_set = False
                else:
                    observation.append(0.0)

                angle_sum += angle_step_size

        return observation, reward, done, ""

    def reset(self):
        self._fitness_data.reset()
        x, y, orientation = self._fitness_data.get_robot_start()
        self._env.set_robot_pose(x, y, orientation)
        return self.step(0.0, 0.0)

    def set_mode(self, mode, terminate_at_end=True):
        self._fitness_data.set_mode(mode, terminate_at_end)