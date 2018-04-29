# -*- coding: utf-8 -*-
import random
import numpy as np
import time
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

from py_agent import EnvironmentGazebo

EPISODES = 10000


class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=5000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(512, input_dim=self.state_size, activation='relu'))
        model.add(Dense(256, input_dim=self.state_size, activation='relu'))
        model.add(Dense(128, activation='relu'))
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


def convert_action(action):
    angular = 0
    linear = 0

    if action == 0:
        angular = 1
        linear = 1.5
    elif action == 1:
        angular = 0.75
        linear = 2
    elif action == 2:
        angular = 0
        linear = 2.5
    elif action == 3:
        angular = -0.75
        linear = 2
    else:
        angular = -1
        linear = 1.5


    return linear, angular

if __name__ == "__main__":
    env = EnvironmentGazebo()
    env.init()
    state_size = 1081
    action_size = 5
    agent = DQNAgent(state_size, action_size)
    # agent.load("./save/cartpole-dqn.h5")
    done = False
    batch_size = 32

    for e in range(EPISODES):

        reward_sum = 0;

        state, _, _, _ = env.step(0.0, 0.0)

	
        state = np.reshape(state, [1, state_size])


        for time in range(500):
            action = agent.act(state)

            linear, angular = convert_action(action)

            next_state, reward, done, _ = env.step(linear, angular)

            next_state = np.reshape(next_state, [1, state_size])

            reward_sum = reward_sum + reward

            agent.remember(state, action, reward_sum, next_state, done)
            state = next_state
            if done:
                print("episode: {}/{}, score: {}, e: {:.2}"
                      .format(e, EPISODES, reward_sum, agent.epsilon))
                break
        if len(agent.memory) > batch_size:
            agent.replay(batch_size)
        if e % 1000 == 0:
             agent.save("./save/dqn" + str(e) + ".h5")
