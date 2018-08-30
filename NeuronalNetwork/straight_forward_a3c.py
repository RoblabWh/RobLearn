'''
Implementierung von A3C mit einem "straight forward" net
'''
import multiprocessing
import threading
import tensorflow as tf
import numpy as np
import random
import os
import shutil
from copy import deepcopy
from tensorflow.python.saved_model import tag_constants

from environment.environment import Environment

# Set to use a saved global net
RESTORE_SAVED_GLOBAL = False
SAVED_GLOBAL_NAME = "test5"

# Set to save the global net every SAVE_INTERVAL episode
SAVE_GLOBAL = True
SAVE_INTERVAL = 10000

# Set to activate visualisation
VISUALIZE = True
# Set to use multiple rooms for the different threads
MULTIPLE_ROOMS = False

NET_NAME = "straight_forward_1"
OUTPUT_GRAPH = True
LOG_DIR = './log'
NET_LOG_DIR = "./net_log/nets/"
NET_DIR = NET_LOG_DIR + NET_NAME + "/" + NET_NAME + ".ckpt"
NET_DIR_SAVED = NET_LOG_DIR + SAVED_GLOBAL_NAME + "/" + SAVED_GLOBAL_NAME + ".ckpt"

N_WORKERS = multiprocessing.cpu_count() - 1  # Number of workers
# N_WORKERS = 2

MAX_EP_STEP = 200  # 200
MAX_GLOBAL_EP = 150000  # 150000  # 1500 150000
GLOBAL_NET_SCOPE = 'Global_Net'
UPDATE_GLOBAL_ITER = 10

GAMMA = 0.9
ENTROPY_BETA = 0.01
LR_A = 0.0001    # 0.0001    # learning rate for actor
LR_C = 0.001    # learning rate for critic
GLOBAL_RUNNING_R = []
GLOBAL_EP = 0

ENV_NAME = "square"
ENV_NAME_2 = "roblab"
ENV_NAME_3 = "room"

CLUSTER_SIZE = 10
SKIP_LRF = 20

env = Environment(ENV_NAME)
env.set_cluster_size(CLUSTER_SIZE)

N_S = env.observation_size() + 64  # state_size  TODO
N_A = 5  # action size


class ACNet(object):
    def __init__(self, scope, globalAC=None):

        if scope == GLOBAL_NET_SCOPE:   # get global network
            with tf.variable_scope(scope):
                self.s = tf.placeholder(tf.float32, [None, N_S], 'S')
                self.a_params, self.c_params = self._build_net(scope)[-2:]
        else:   # local net, calculate losses
            with tf.variable_scope(scope):
                self.s = tf.placeholder(tf.float32, [None, N_S], 'S')
                self.a_his = tf.placeholder(tf.float32, [None, N_A], 'A')
                self.v_target = tf.placeholder(tf.float32, [None, 1], 'Vtarget')

                mu, sigma, self.v, self.a_params, self.c_params = self._build_net(scope)
                td = tf.subtract(self.v_target, self.v, name='TD_error')  # Value loss / critic loss

                with tf.name_scope('c_loss'):
                    self.c_loss = tf.reduce_mean(tf.square(td))

                with tf.name_scope('wrap_a_out'):
                    # mu, sigma = mu * random.randint(-300, 301), sigma + 1e-4
                    mu, sigma = mu * random.uniform(0, 1), sigma + 1e-4
                    # mu, sigma = mu * A_BOUND[1], sigma + 1e-4

                normal_dist = tf.distributions.Normal(mu, sigma)

                with tf.name_scope('a_loss'):
                    log_prob = normal_dist.log_prob(self.a_his)
                    exp_v = log_prob * tf.stop_gradient(td)
                    entropy = normal_dist.entropy()  # encourage exploration
                    self.exp_v = ENTROPY_BETA * entropy + exp_v
                    self.a_loss = tf.reduce_mean(-self.exp_v) # Policy loss / actor loss

                with tf.name_scope('choose_a'):  # use local params to choose action
                    self.A = tf.reshape(normal_dist.sample(1), [1, 5])

                with tf.name_scope('local_grad'):
                    self.a_grads = tf.gradients(self.a_loss, self.a_params)
                    self.c_grads = tf.gradients(self.c_loss, self.c_params)

            with tf.name_scope('sync'):
                with tf.name_scope('pull'):
                    self.pull_a_params_op = [l_p.assign(g_p) for l_p, g_p in zip(self.a_params, globalAC.a_params)]
                    self.pull_c_params_op = [l_p.assign(g_p) for l_p, g_p in zip(self.c_params, globalAC.c_params)]
                with tf.name_scope('push'):
                    self.update_a_op = OPT_A.apply_gradients(zip(self.a_grads, globalAC.a_params))
                    self.update_c_op = OPT_C.apply_gradients(zip(self.c_grads, globalAC.c_params))

        # Add ops to save and restore all the variables.
        self.saver = tf.train.Saver()

    def _build_net(self, scope):
        w_init = tf.random_normal_initializer(0., .1)

        # # Neural Net for Deep-Q learning Model
        # model = Sequential()
        # model.add(Dense(2048, input_dim=self.state_size, activation='relu'))
        # model.add(Dense(512, activation='relu'))
        # model.add(Dense(256, activation='relu'))
        # model.add(Dense(self.action_size, activation='linear'))
        # model.compile(loss='mse',
        #               optimizer=Adam(lr=self.learning_rate))
        # return mod

        with tf.variable_scope('critic'):   # only critic controls the rnn update
            # s = tf.expand_dims(self.s, axis=1,
            #                    name='timely_input')  # [time_step, feature] => [time_step, batch, feature]
            #
            # # create 2 LSTMCells
            # rnn_layers = [tf.nn.rnn_cell.LSTMCell(size) for size in [128, 256]]
            #
            # # create a RNN cell composed sequentially of a number of RNNCells
            # multi_rnn_cell = tf.nn.rnn_cell.MultiRNNCell(rnn_layers)
            #
            # self.init_state = multi_rnn_cell.zero_state(batch_size=1, dtype=tf.float32)
            #
            # # 'outputs' is a tensor of shape [batch_size, max_time, 256]
            # # 'self.final_state' is a N-tuple where N is the number of LSTMCells containing a
            # # tf.contrib.rnn.LSTMStateTuple for each cell
            # outputs, self.final_state = tf.nn.dynamic_rnn(cell=multi_rnn_cell,
            #                                    inputs=s,
            #                                    dtype=tf.float32)

            # cell_out = tf.reshape(outputs, [-1, 256], name='flatten_rnn_outputs')  # joined state representation

            layer1 = tf.layers.dense(self.s, 1024, tf.nn.relu6)
            layer2 = tf.layers.dense(layer1, 512, tf.nn.relu6)
            layer3 = tf.layers.dense(layer2, 256, tf.nn.relu6)

            l_c = tf.layers.dense(layer3, 50, tf.nn.relu6, kernel_initializer=w_init, name='lc')

            v = tf.layers.dense(l_c, 1, kernel_initializer=w_init, name='v')  # state value

        with tf.variable_scope('actor'):  # state representation is based on critic
            l_a = tf.layers.dense(layer3, 80, tf.nn.relu6, kernel_initializer=w_init, name='la')
            mu = tf.layers.dense(l_a, N_A, tf.nn.tanh, kernel_initializer=w_init, name='mu')
            sigma = tf.layers.dense(l_a, N_A, tf.nn.softplus, kernel_initializer=w_init, name='sigma')
        a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/actor')
        c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/critic')

        return mu, sigma, v, a_params, c_params

    def update_global(self, feed_dict):  # run by a local
        SESS.run([self.update_a_op, self.update_c_op], feed_dict)  # local grads applies to global net

    def pull_global(self):  # run by a local
        SESS.run([self.pull_a_params_op, self.pull_c_params_op])

    # def choose_action(self, s, cell_state):  # run by a local
    def choose_action(self, s):  # run by a local
        # a, cell_state = SESS.run([self.A], {self.s: s, self.init_state: cell_state})
        a = SESS.run([self.A], {self.s: s})
        return a
        # return a, cell_state

    def save_global(self):
        # TODO
        self.saver.save(SESS, NET_DIR)
        # inputs = {"s": self.s}
        # outputs = {"v": self.v}
        # tf.saved_model.simple_save(SESS, NET_DIR, inputs, outputs)


class Worker(object):
    def __init__(self, name, globalAC):
        if MULTIPLE_ROOMS:
            if name == "W_0" or name == "W_1" or name == "W_2":
                self.env = Environment(ENV_NAME)
            elif name == "W_3" or name == "W_4" or name == "W_5":
                self.env = Environment(ENV_NAME_2)
            else:
                self.env = Environment(ENV_NAME_3)
        else:
            self.env = Environment(ENV_NAME)

        self.env.set_cluster_size(CLUSTER_SIZE)
        self.env.set_observation_rotation_size(64)  # TODO
        self.env.use_observation_rotation_size(True)
        self.name = name
        self.AC = ACNet(name, globalAC)

    def convert_action(self, action):
        angular = 0
        linear = 0

        if action == 0:
            angular = 1.0
            linear = 0.5
        elif action == 1:
            angular = 0.5
            linear = 0.75
        elif action == 2:
            angular = 0.0
            linear = 1.0
        elif action == 3:
            angular = -0.5
            linear = 0.75
        else:
            angular = -1.0
            linear = 0.5

        return linear, angular

    def work(self):
        global GLOBAL_RUNNING_R, GLOBAL_EP
        total_step = 1
        buffer_s, buffer_a, buffer_r = [], [], []
        while not COORD.should_stop() and GLOBAL_EP < MAX_GLOBAL_EP:
            s, _, _, _ = self.env.reset()
            s = np.reshape(s, [1, N_S])
            ep_r = 0
            # rnn_state = SESS.run(self.AC.init_state)    # zero rnn state at beginning
            # keep_state = deepcopy(rnn_state)      # keep rnn state for updating global net
            for ep_t in range(MAX_EP_STEP):

                # a, rnn_state_ = self.AC.choose_action(s, rnn_state)  # get the action and next rnn state
                a = self.AC.choose_action(s)  # get the action and next rnn state
                b = np.asarray(a)
                b = b[0][0]

                action = np.argmax(b)

                linear, angular = self.convert_action(action)

                s_, r, done, _ = self.env.step(linear, angular, SKIP_LRF)
                s_ = np.reshape(s_, [1, N_S])

                # if (self.name == 'W_0' or self.name == "W_3") and VISUALIZE:
                if (self.name == 'W_0') and VISUALIZE:
                    self.env.visualize()

                done = True if ep_t == MAX_EP_STEP - 1 else done

                ep_r += r
                buffer_s.append(s)
                buffer_a.append(b)
                buffer_r.append(r)
                # buffer_r.append((r+8)/8)    # normalize

                if total_step % UPDATE_GLOBAL_ITER == 0 or done:   # update global and assign to local net
                    if done:
                        v_s_ = 0   # terminal
                    else:
                        # v_s_ = SESS.run(self.AC.v, {self.AC.s: s_, self.AC.init_state: rnn_state_})[0, 0]
                        v_s_ = SESS.run(self.AC.v, {self.AC.s: s_})[0, 0]
                    buffer_v_target = []
                    for r in buffer_r[::-1]:    # reverse buffer r
                        v_s_ = r + GAMMA * v_s_
                        buffer_v_target.append(v_s_)
                    buffer_v_target.reverse()

                    buffer_s, buffer_a, buffer_v_target = np.vstack(buffer_s), np.vstack(buffer_a), np.vstack(buffer_v_target)
                    feed_dict = {
                        self.AC.s: buffer_s,
                        self.AC.a_his: buffer_a,
                        self.AC.v_target: buffer_v_target,
                        # self.AC.init_state: keep_state,
                    }

                    self.AC.update_global(feed_dict)
                    buffer_s, buffer_a, buffer_r = [], [], []
                    self.AC.pull_global()

                    # keep_state = deepcopy(rnn_state_)   # replace the keep_state as the new initial rnn state_

                s = s_
                # rnn_state = rnn_state_  # renew rnn state
                total_step += 1

                if done:
                    if len(GLOBAL_RUNNING_R) == 0:  # record running episode reward
                        GLOBAL_RUNNING_R.append(ep_r)
                    else:
                        GLOBAL_RUNNING_R.append(0.9 * GLOBAL_RUNNING_R[-1] + 0.1 * ep_r)

                    if self.name == "W_0":
                        print(self.name, "Ep:", GLOBAL_EP, "Ep_r:", ep_r)
                        # print(
                        #     self.name,
                        #     "Ep:", GLOBAL_EP,
                        #     "| Ep_r: %i" % GLOBAL_RUNNING_R[-1],
                        #       )
                    GLOBAL_EP += 1
                    if GLOBAL_EP % SAVE_INTERVAL == 0:
                        print("Versuche zu Speichern...")
                        self.AC.save_global()
                        print("...gespeichert!")
                    break


if __name__ == "__main__":
    SESS = tf.Session()

    with tf.device("/cpu:0"):
        # OPT_A = tf.train.RMSPropOptimizer(LR_A, name='RMSPropA')
        # OPT_C = tf.train.RMSPropOptimizer(LR_C, name='RMSPropC')
        OPT_A = tf.train.AdamOptimizer(LR_A, name='AdamA')
        OPT_C = tf.train.AdamOptimizer(LR_C, name='AdamC')

        GLOBAL_AC = ACNet(GLOBAL_NET_SCOPE)  # we only need its params

        if RESTORE_SAVED_GLOBAL:
            print("Restore Net:", NET_DIR_SAVED)
            GLOBAL_AC.saver.restore(SESS, NET_DIR_SAVED)
            print("...restored!")

        workers = []
        # Create worker
        for i in range(N_WORKERS):
            i_name = 'W_%i' % i   # worker name
            workers.append(Worker(i_name, GLOBAL_AC))

    COORD = tf.train.Coordinator()
    SESS.run(tf.global_variables_initializer())

    if OUTPUT_GRAPH:
        if os.path.exists(LOG_DIR):
            shutil.rmtree(LOG_DIR)
        tf.summary.FileWriter(LOG_DIR, SESS.graph)

    worker_threads = []
    for worker in workers:
        job = lambda: worker.work()
        t = threading.Thread(target=job)
        t.start()
        worker_threads.append(t)
    COORD.join(worker_threads)



