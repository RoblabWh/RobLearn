#! /usr/bin/env python3
import socket
import struct

import sys

import time

import random

from ga3c.Config import Config

from action_getter import ActionGetter
from nn_interface.nn_interface import NN_Interface

def main():
    nn_interface = NN_Interface()
    nn_interface.init()

    ag = ActionGetter()

    observations = []

    while True:
        # Get the observation from the laserscan
        observation = nn_interface.receive_observation()

        if len(observations) < 4:
            observations.append(observation)
        else:
            observations[0] = observations[1]
            observations[1] = observations[2]
            observations[2] = observations[3]
            observations[3] = observation

            # TODO: Get the action from observation
            # NN get action
            # linear, angular = ag.act(observation)
            linear, angular = ag.get_action(observations[0],observations[1],observations[2],observations[3])

            # Send the action to the robot
            # nn_interface.send_action(random.uniform(0.0, 0.5), random.uniform(-1, 1))
            nn_interface.send_action(linear, angular)


if __name__ == '__main__':
    main()