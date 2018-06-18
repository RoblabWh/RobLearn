from random import random, randrange

import numpy as np
import tensorflow as tf
import tflearn
from tflearn import Evaluator, DNN

import action_mapper
from environment import Environment

MAX_EPISODES = 100000
gamma = 0.99
# Number of timesteps to anneal epsilon
anneal_epsilon_timesteps = 400000

final_epsilon = 0.01

visualize = False



class NeuralNet(object):

    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.input, self.model = self._build_graph()
        self.network_params = tf.trainable_variables()

        self.target_input, self.target_model = self._build_graph()
        self.target_network_params = tf.trainable_variables()[len(self.network_params):]

        self.update = self._graph_update()

        self.reset_target_net = \
            [self.target_network_params[i].assign(self.network_params[i]) for i in range(len(self.target_network_params))]

    def _graph_update(self):
        self.updater_a = tf.placeholder('float', [None, self.action_size])
        self.updater_y = tf.placeholder('float', [None])
        action_q_values = tf.reduce_sum(tf.multiply(self.model, self.updater_a), reduction_indices=1)
        cost = tflearn.mean_square(action_q_values, self.updater_y)
        optimizer = tf.train.RMSPropOptimizer(0.001)
        grad_update = optimizer.minimize(cost, var_list=self.network_params)
        return grad_update

    def _build_graph(self):
        input = tflearn.layers.input_data(shape=(None, self.state_size), dtype=tf.float32)
        print(input.shape)
        input = tf.expand_dims(input, -1)
        print(input.shape)
        net = input
        # net = tflearn.layers.conv_1d(net, 16, 3, padding='same')
        # net = tflearn.layers.max_pool_1d(net, 3)
        # net = tflearn.layers.conv_1d(net, 16, 2)
        # net = tflearn.layers.max_pool_1d(net, 2)
        net = tflearn.layers.fully_connected(net, 64, activation='relu')
        net = tflearn.layers.fully_connected(net, self.action_size, activation='relu')
        return input, net

    # def step(self, obs):
    #     ev = Evaluator(self.model)
    #     ev.evaluate()

    def train(self, session, env):
        session.run(tf.initialize_all_variables())

        num_episode = 0

        initial_epsilon = 1.0
        epsilon = initial_epsilon

        while num_episode <= MAX_EPISODES:
            reset_env(env)
            state, _, _, _ = env.step(0, 0)
            state = np.reshape(state, [1, 54, 1])

            num_episode += 1

            episode_step = 0
            episode_reward = 0
            while True:
                q_values = self.model.eval(session=session, feed_dict={self.input: state})

                if random() <= epsilon:
                    action_index = randrange(self.action_size)
                else:
                    action_index = np.argmax(q_values)

                a_t = np.zeros([self.action_size])
                a_t[action_index] = 1

                if epsilon > final_epsilon:
                    epsilon -= (initial_epsilon - final_epsilon) / anneal_epsilon_timesteps

                #print("Choosing Action {}".format(action_index))

                x1, x2 = action_mapper.map_action(action_index)
                next_state, reward, term, info = env.step(x1, x2)
                next_state = np.reshape(next_state, [1, 54, 1])
                episode_reward += reward

                if visualize:
                    env.visualize()

                #print("Reward: {} \n\n".format(reward))

                next_q_values = self.target_model.eval(session=session, feed_dict={self.target_input: next_state})

                if not term:
                    reward = reward + gamma * np.argmax(next_q_values)

                if num_episode % 1000 == 0:
                    session.run(self.reset_target_net)
                    print("Target Net Resetted")

                session.run(self.update, feed_dict={self.updater_y: [reward],
                                                    self.updater_a: [a_t],
                                                    self.input: state})

                if term:
                    break

                state = next_state
                episode_step += 1

            print("Episode {} Terminated. Rewardsum: {}".format(num_episode, episode_reward))

# def train():
#     env = Environment()
#
#     env.init()
#     obs = env.step(0, 0)
#     print(obs)
#
#     while True:
#         env.step(0, -1)


def reset_env(env):
    env.set_start(-9.5, -9.5, 1.57079632679)
    env.set_target(9, 9)


if __name__ == '__main__':

    env = Environment()
    env.set_cluster_size(20)

    net = NeuralNet(state_size=env.observation_size(), action_size=action_mapper.ACTION_SIZE)

    with tf.Session(config=tf.ConfigProto(intra_op_parallelism_threads=4)) as sess:
        net.train(sess, env)
