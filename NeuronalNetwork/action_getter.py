#!/usr/bin/env python3

# import DQNAgent
import numpy as np

from environment.environment import Environment
from environment.environment_node_data import Mode

from ga3c.NetworkVP import NetworkVP
from ga3c.Config import Config

import action_mapper

NET_TO_LOAD = "dqn_3_2500"
ACTION_SIZE = 5
STATE_SIZE = 1081


class ActionGetter:

    def __init__(self, state_size, action_size):
        # self.agent = DQNAgent(STATE_SIZE, ACTION_SIZE)
        # self.agent.load("./save/" + NET_TO_LOAD + ".h5")

        # self.model = NetworkVP(Config.DEVICE, Config.NETWORK_NAME, Environment().get_num_actions())
        self.model = NetworkVP(Config.DEVICE, Config.NETWORK_NAME, ACTION_SIZE)

        self.model.load()


    def get_action(self, state1, state2=None, state3=None, state4=None):
        # action = self.agent.act(state1)
        # linear, angular = action_mapper.map_action(action)

        state = [state1, state2, state3, state4]

        state = np.reshape(state, [Config.OBSERVATION_SIZE, Config.STACKED_FRAMES])

        action = self.model.predict_single(state)

        linear, angular = action_mapper.map_action(np.argmax(action))

        return linear, angular

if __name__ == '__main__':
    ag = ActionGetter(STATE_SIZE, ACTION_SIZE)

    test1 = np.zeros(Config.OBSERVATION_SIZE)
    test2 = np.zeros(Config.OBSERVATION_SIZE)
    test3 = np.zeros(Config.OBSERVATION_SIZE)
    test4 = np.zeros(Config.OBSERVATION_SIZE)

    print("action: ", ag.get_action(test1, test2, test3, test4))