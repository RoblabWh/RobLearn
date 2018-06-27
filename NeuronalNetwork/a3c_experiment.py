import threading
import numpy as np
import tensorflow as tf
from matplotlib import pylab
import time
import random
from keras.layers import Dense, Input
from keras.models import Model
from keras.optimizers import Adam
from keras import backend as K
from environment.environment import Environment



episode = 0
scores = []

EPISODES = 10000

class A3CAgent:
    def __init__(self, state_size, action_size):

        self.state_size = state_size
        self.action_size = action_size

        self.actor_lr = 0.001
        self.critic_lr = 0.001
        self.discount_factor = .99
        self.hidden1, self.hidden2 = 24, 24
        self.threads = 8

        self.actor, self.critic = self.build_model()

        self.optimizer = [self.actor_optimizer(), self.critic_optimizer()]

        self.sess = tf.InteractiveSession()
        K.set_session(self.sess)
        self.sess.run(tf.global_variables_initializer())

    def build_model(self):
        state = Input(batch_shape=(None,  self.state_size))
        print ("state:" , state)
        shared = Dense(self.hidden1, input_dim=self.state_size, activation='relu', kernel_initializer='glorot_uniform')(state)

        actor_hidden = Dense(self.hidden2, activation='relu', kernel_initializer='glorot_uniform')(shared)
        action_prob = Dense(self.action_size, activation='softmax', kernel_initializer='glorot_uniform')(actor_hidden)

        value_hidden = Dense(self.hidden2, activation='relu', kernel_initializer='he_uniform')(shared)
        state_value = Dense(1, activation='linear', kernel_initializer='he_uniform')(value_hidden)

        actor = Model(inputs=state, outputs=action_prob)
        critic = Model(inputs=state, outputs=state_value)

        actor._make_predict_function()
        critic._make_predict_function()

        actor.summary()
        critic.summary()

        return actor, critic

    
    def actor_optimizer(self):
        action = K.placeholder(shape=(None, self.action_size))
        advantages = K.placeholder(shape=(None, ))

        policy = self.actor.output

        good_prob = K.sum(action * policy, axis=1)
        eligibility = K.log(good_prob + 1e-10) * K.stop_gradient(advantages)
        loss = -K.sum(eligibility)

        entropy = K.sum(policy * K.log(policy + 1e-10), axis=1)

        actor_loss = loss + 0.01*entropy

        optimizer = Adam(lr=self.actor_lr)
        updates = optimizer.get_updates(self.actor.trainable_weights, [], actor_loss)
        train = K.function([self.actor.input, action, advantages], [], updates=updates)
        return train

    def critic_optimizer(self):
        discounted_reward = K.placeholder(shape=(None, ))

        value = self.critic.output

        loss = K.mean(K.square(discounted_reward - value))

        optimizer = Adam(lr=self.critic_lr)
        updates = optimizer.get_updates(self.critic.trainable_weights, [], loss)
        train = K.function([self.critic.input, discounted_reward], [], updates=updates)
        return train


    def train(self):

        agents = [Agent(i, self.actor, self.critic, self.optimizer, self.discount_factor,
                        self.action_size, self.state_size) for i in range(self.threads)]

        for agent in agents:
            agent.start()

        while True:
            time.sleep(20)

            plot = scores[:]
            pylab.plot(range(len(plot)), plot, 'b')
            pylab.savefig("./save_graph/AR_a3c_Exp.png")

            self.save_model('./save_model/AR_a3c_Exp.h5')

    def save_model(self, name):
        self.actor.save_weights(name + "_actor.h5")
        self.critic.save_weights(name + "_critic.h5")

    def load_model(self, name):
        self.actor.load_weights(name + "_actor.h5")
        self.critic.load_weights(name + "_critic.h5")


class Agent(threading.Thread):
    def __init__(self, index, actor, critic, optimizer, discount_factor, action_size, state_size):
        threading.Thread.__init__(self)

        self.states = []
        self.rewards = []
        self.actions = []

        self.index = index
        self.actor = actor
        self.critic = critic
        self.optimizer = optimizer

        self.discount_factor = discount_factor
        self.action_size = action_size
        self.state_size = state_size
    def convert_action(self, action):
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


    def run(self):
        global episode
        env = Environment("test")
        while episode < EPISODES:
            state, _, _, _ = env.reset()
            score = 0
            while True:
                action = self.get_action(state)
                print ("action:", action)
                linear, angular = self.convert_action(action)
                next_state, reward, done, _ = env.step(linear, angular, 10)
                next_state = np.reshape(next_state, [1, state_size])
                score += reward

                self.memory(state, action, reward)

                state = next_state
                env.visualize()

                if done:
                    episode += 1
                    print("episode: ", episode, "/ score : ", score)
                    scores.append(score)
                    self.train_episode(score != 500)
                    break


    def discount_rewards(self, rewards, done=True):
        discounted_rewards = np.zeros_like(rewards)
        running_add = 0
        if not done:
            running_add = self.critic.predict(np.reshape(self.states[-1], (1, self.state_size)))[0]
        for t in reversed(range(0, len(rewards))):
            running_add = running_add * self.discount_factor + rewards[t]
            discounted_rewards[t] = running_add
        return discounted_rewards


    def memory(self, state, action, reward):
        self.states.append(state)
        act = np.zeros(self.action_size)
        act[action] = 1
        self.actions.append(act)
        self.rewards.append(reward)

    # update policy network and value network every episode
    def train_episode(self, done):
        discounted_rewards = self.discount_rewards(self.rewards, done)
        values = self.critic.predict(np.array(self.states))
        print ("self.state:" , self.states)
        values = np.reshape(values, len(values))


        advantages = discounted_rewards - values

        self.optimizer[0]([self.states, self.actions, advantages])
        self.optimizer[1]([self.states, discounted_rewards])
        self.states, self.actions, self.rewards = [], [], []

    def get_action(self, state):
        policy = self.actor.predict(np.reshape(state, [1, self.state_size]))[0]
        return np.random.choice(self.action_size, 1, p=policy)[0]


if __name__ == "__main__":

    env = Environment("test")

    state_size = env.observation_size()
    action_size = 5


    global_agent = A3CAgent(state_size, action_size)
    
global_agent.train()
