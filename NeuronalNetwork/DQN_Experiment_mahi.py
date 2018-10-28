import gym
import numpy as np
import random
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import Adam

from collections import deque
from environment.environment import Environment

class DQN:
    def __init__(self, env):
        self.env     = env
        self.memory  = deque(maxlen=5000)
        
        self.gamma = 0.85
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.005
        self.tau = .125

        self.model        = self.create_model()
        self.target_model = self.create_model()

    def create_model(self):
        model   = Sequential()
        state_size  = self.env.observation_size()
        action_size = 5
        model.add(Dense(24, input_dim=state_size, activation="relu"))
        model.add(Dense(48, activation="relu"))
        model.add(Dense(24, activation="relu"))
        model.add(Dense(action_size))
        model.compile(loss="mean_squared_error",
            optimizer=Adam(lr=self.learning_rate))
        return model

    def act(self, state):
        self.epsilon *= self.epsilon_decay
        self.epsilon = max(self.epsilon_min, self.epsilon)
        if np.random.random() < self.epsilon:
            return random.randrange(5)
        return np.argmax(self.model.predict(state)[0])

    def remember(self, state, action, reward, new_state, done):
        self.memory.append([state, action, reward, new_state, done])

    def replay(self):
        batch_size = 32
        if len(self.memory) < batch_size: 
            print(" memory is smaller than batch")
            return
        samples = random.sample(self.memory, batch_size)
        for sample in samples:
            state, action, reward, new_state, done = sample
            target = self.target_model.predict(state)
            if done:
                target[0][action] = reward
            else:
                Q_future = max(self.target_model.predict(new_state)[0])
                target[0][action] = reward + Q_future * self.gamma
            self.model.fit(state, target, epochs=1, verbose=0)

    def target_train(self):
        weights = self.model.get_weights()
        target_weights = self.target_model.get_weights()
        for i in range(len(target_weights)):
            target_weights[i] = weights[i] * self.tau + target_weights[i] * (1 - self.tau)
        self.target_model.set_weights(target_weights)

    def save_model(self, fn):
        self.model.save(fn)

def main():
    #env     = gym.make("MountainCar-v0")
    env     = Environment("test")
    state_size = env.observation_size()
    gamma   = 0.9
    epsilon = .95

    trials  = 1000
    trial_len = 500

    # updateTargetNetwork = 1000
    dqn_agent = DQN(env=env)
    done = False
    batch_size = 32
    steps = []
    for trial in range(trials):

        reward_sum = 0
        cur_state, _, _, _ = env.reset()
        cur_state = np.reshape(cur_state, [1, state_size])
        for step in range(trial_len):
            action = dqn_agent.act(cur_state)
            linear , angular = convert_action(action)

            new_state, reward, done, _ = env.step(linear, angular, 10)
            # reward = reward if not done else -20
            new_state = np.reshape(new_state, [1, state_size])
            reward_sum = reward_sum + reward
            dqn_agent.remember(cur_state, action, reward, new_state, done)
            
            #dqn_agent.replay()       # internally iterates default (prediction) model
            dqn_agent.target_train() # iterates target model

            cur_state = new_state

            env.visualize()
            if done:
                print("episode: {}/{}, score: {}, e: {:.2} time:{}"
                      .format(trial, trials, reward_sum, dqn_agent.epsilon, step))
                break
        if len(dqn_agent.memory) > batch_size:
            dqn_agent.replay()

             
def convert_action(action):
    angular = 0
    linear = 0

    if action == 0:
        angular = 0.77
        linear = 0.75
    elif action == 1:
        angular = 0.44
        linear = 1.25
    elif action == 2:
        angular = 0
        linear = 1.5
    elif action == 3:
        angular = -0.44
        linear = 1.25
    else:
        angular = -0.77
        linear = 0.75

    return linear, angular

if __name__ == "__main__":
    main()
