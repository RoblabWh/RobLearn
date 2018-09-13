# -*- coding: utf-8 -*-
import random
import numpy as np
import time
from collections import deque
from keras.models import Sequential
from keras.layers import Dense, TimeDistributed
from keras.layers import SimpleRNN
from keras.optimizers import Adam

from environment.environment import Environment
from environment.environment_node_data import Mode
import action_mapper 

EPISODES = 10000


class DQNAgent:

    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=50000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0  # exploration rate
        # self.epsilon_min = 0.01
        # self.epsilon_decay = 0.995
        # self.learning_rate = 0.001
        self.epsilon_min = 0.0
        self.epsilon_decay = 0.998
        self.learning_rate = 0.001

        self.model = self._build_model()

    def _build_model(self):

        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(2048, input_dim=self.state_size, activation='relu'))
        model.add(Dense(512, activation='relu'))
        model.add(Dense(256, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma *
                          np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)



if __name__ == "__main__":
    env = Environment("../Simulation2d/world/test")
    #env.set_mode(Mode.PAIR_ALL, terminate_at_end=True)
    #env.set_mode(Mode.ALL_RANDOM, terminate_at_end=False)
    env.use_observation_rotation_size(True)
    #env.set_cluster_size(10)
    env.set_observation_rotation_size(128)

    state_size = env.observation_size()
    action_size = action_mapper.ACTION_SIZE
    agent = DQNAgent(state_size, action_size)
    # agent.load("./save/cartpole-dqn.h5")
    done = False
    batch_size = 48

    print("START DQN")

    for e in range(EPISODES):

        visualize = (e % 5 == 0)

        reward_sum = 0

        state, _, _, _ = env.reset()

        state = np.reshape(state, [1, state_size])

        for iteration in range(100):
            action = agent.act(state)

            linear, angular = action_mapper.map_action(action)

            next_state, reward, done, _ = env.step(linear, angular, 20)

            next_state = np.reshape(next_state, [1, state_size])

            reward_sum = reward_sum + reward

            agent.remember(state, action, reward_sum, next_state, done)
            state = next_state

            if visualize:
                env.visualize()
                #time.sleep(1.0)

            if done:
                print("episode: {}/{}, score: {}, e: {:.2} iteration:{}"
                      .format(e, EPISODES, reward_sum, agent.epsilon, iteration))
                break
        if len(agent.memory) > batch_size:
            agent.replay(batch_size)
        #if e % 1000 == 0:
        #     agent.save("./save/dqn" + str(e) + ".h5")
