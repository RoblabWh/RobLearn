import time
from random import random, randrange

import numpy as np
import tensorflow as tf
import tflearn
from tflearn import Evaluator, DNN

import action_mapper
from environment.environment import Environment

flags = tf.flags

flags.DEFINE_boolean('evaluate', False, 'If true, run the rob evaluation instead of training.')
flags.DEFINE_string('checkpoint_path', '', 'Path to the checkpoint to evaluate.')
flags.DEFINE_integer('evaluation_episodes', 50, '')
flags.DEFINE_float('sleep', 0.1, 'Sleep in seconds between steps in evaluation.')
FLAGS = flags.FLAGS

MAX_EPISODES = 100000
LOG_PATH = './log/'
CHECKPOINT_PATH = './save/N-Step-CNN.ckpt'
CHECKPOINT_PERIOD_TIMESTEPS = 10000
target_update_timestep = 2000
gamma = 0.99
# Number of timesteps to anneal epsilon
anneal_epsilon_timesteps = 400000

final_epsilon = 0.02

visualize = True


class NeuralNet(object):

    def __init__(self, state_size, action_size, writer):
        self.state_size = state_size
        self.action_size = action_size
        self.writer = writer

        self.input, self.model = self._build_graph()
        self.network_params = tf.trainable_variables()

        self.target_input, self.target_model = self._build_graph()
        self.target_network_params = tf.trainable_variables()[len(self.network_params):]

        self.update = self._graph_update()

        self.reset_target_net = \
            [self.target_network_params[i].assign(self.network_params[i]) for i in range(len(self.target_network_params))]

        self.saver = tf.train.Saver(max_to_keep=10, )

    def _graph_update(self):
        self.updater_a = tf.placeholder('float', [None, self.action_size])
        self.updater_y = tf.placeholder('float', [None])
        action_q_values = tf.reduce_sum(tf.multiply(self.model, self.updater_a), reduction_indices=1)
        cost = tflearn.mean_square(action_q_values, self.updater_y)
        # optimizer = tf.train.RMSPropOptimizer(0.001)
        optimizer = tf.train.AdamOptimizer(0.002)
        grad_update = optimizer.minimize(cost, var_list=self.network_params)
        return grad_update

    def _build_graph(self):
        input = tflearn.layers.input_data(shape=(None, self.state_size), dtype=tf.float32)
        input = tf.expand_dims(input, -1)
        net = input
        net = tflearn.layers.conv_1d(net, 16, 3, padding='same')
        net = tflearn.layers.max_pool_1d(net, 3)
        net = tflearn.layers.conv_1d(net, 16, 2)
        net = tflearn.layers.max_pool_1d(net, 2)
        net = tflearn.layers.fully_connected(net, 64, activation='relu')
        net = tflearn.layers.fully_connected(net, self.action_size, activation='linear')
        # net = tflearn.layers.fully_connected(net, 512, activation='relu')
        # net = tflearn.layers.fully_connected(net, 256, activation='relu')
        # net = tflearn.layers.fully_connected(net, self.action_size, activation='linear')
        return input, net

    def train(self, session, env):
        session.run(tf.initialize_all_variables())

        num_episode = 0
        global_step = 0

        initial_epsilon = 1.0
        epsilon = initial_epsilon

        while num_episode <= MAX_EPISODES:
            reset_env(env)
            state, _, _, _ = env.step(0, 0)
            state = np.reshape(state, [1, self.state_size, 1]) # @TODO Auslagern

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
                next_state, reward, term, info = env.step(x1, x2, 10)
                next_state = np.reshape(next_state, [1, self.state_size, 1])
                episode_reward += reward

                if visualize:
                    env.visualize()

                #print("Reward: {} \n\n".format(reward))

                next_q_values = self.target_model.eval(session=session, feed_dict={self.target_input: next_state})

                if not term:
                    reward = reward + gamma * np.amax(next_q_values)

                if global_step % target_update_timestep == 0:
                    session.run(self.reset_target_net)
                    print("Target Net Resetted")

                weights_old = session.run(self.network_params) ## DEBUG
                session.run(self.update, feed_dict={self.updater_y: [reward],
                                                    self.updater_a: [a_t],
                                                    self.input: state})

                weights_after = session.run(self.network_params) ## DEBUG

                if global_step % CHECKPOINT_PERIOD_TIMESTEPS == 0:
                    self.saver.save(session, CHECKPOINT_PATH, global_step=global_step)

                global_step += 1
                state = next_state
                episode_step += 1

                if term:
                    # weights = session.run(self.network_params) ## DEBUG
                    break

            print("Episode {} Terminated. Rewardsum: {}, Globalstep: {}".format(num_episode, episode_reward, global_step))

    def evaluate(self, session, env):
        self.saver.restore(session, FLAGS.checkpoint_path)
        print('Loaded Model from ', FLAGS.checkpoint_path)

        num_episode = 0
        while num_episode < FLAGS.evaluation_episodes:
            reset_env(env)
            state, _, terminal, _ = env.step(0, 0)
            state = np.reshape(state, [1, self.state_size, 1]) # @TODO Auslagern
            env.visualize()
            episode_reward = 0

            while not terminal:
                q_values = self.model.eval(session=session, feed_dict={self.input: state})

                action = np.argmax(q_values)

                x1, x2 = action_mapper.map_action(action)
                next_state, reward, terminal, info = env.step(x1, x2, 10)
                next_state = np.reshape(next_state, [1, self.state_size, 1])
                env.visualize()

                episode_reward += reward

                state = next_state

                time.sleep(FLAGS.sleep)

            num_episode += 1
            print('Testing Episode finished. Reward: {}'.format(episode_reward))


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
    # Method maybe obsolete?
    env.reset()


if __name__ == '__main__':

    env = Environment('test')
    env.set_cluster_size(36)

    writer = tf.summary.FileWriter(LOG_PATH)

    net = NeuralNet(state_size=env.observation_size(), action_size=action_mapper.ACTION_SIZE, writer=writer)

    with tf.Session(config=tf.ConfigProto(intra_op_parallelism_threads=4)) as sess:
        writer.add_graph(sess.graph)

        if FLAGS.evaluate:
            net.evaluate(sess, env)
        else:
            net.train(sess, env)
