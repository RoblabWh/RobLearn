import tensorflow as tf
import numpy as np
import scipy.signal
from tflearn.utils import feed_dict_builder

from a3c.ACNetwork import ACNetwork
from environment.environment import Environment
import a3c.action_mapper
import time


def copy_tf_vars(source, target):
    from_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, source)
    to_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, target)

    op_holder = []
    for from_var, to_var in zip(from_vars, to_vars):
        op_holder.append(to_var.assign(from_var))
    return op_holder

# Discounting function used to calculate discounted returns.
def discount(x, gamma):
    return scipy.signal.lfilter([1], [1, -gamma], x[::-1], axis=0)[::-1]



class Worker(object):
    def __init__(self, world_name, thread_name, state_size, action_size, trainer, saver, model_path):
        self.name = 'Worker_' + str(thread_name)
        self.state_size = state_size
        self.action_size = action_size
        self.trainer = trainer

        # @TODO Env configuration
        self.env = Environment(world_name)
        self.env.set_cluster_size(30)
        self.env.use_observation_rotation_size(False)

        self.local_AC = ACNetwork(state_size, action_size, self.name, trainer)
        self.update_local_ops = copy_tf_vars('global', self.name) # @TODO Magic String...

        self.episode_rewards = []
        self.episode_lengths = []
        self.episode_mean_values = []

    def work(self, max_episode_length, gamma, session, coordinator, saver):
        episode_count = 0
        total_step_count = 0

        print("Starting worker {}".format(self.name))

        with session.as_default(), session.graph.as_default():
            while not coordinator.should_stop():
                session.run(self.update_local_ops)

                episode_buffer = []
                episode_value = []
                episode_frames = []
                episode_reward = 0
                episode_step_count = 0
                done = False

                state, _, _, _ = self.env.reset()

                while not done:
                    # @TODO Reshape dings
                    state = np.reshape(state, [1, self.state_size])
                    action_dist, value = session.run([self.local_AC.actor, self.local_AC.critic], feed_dict={self.local_AC.input: state})

                    action = np.random.choice(action_dist[0], p=action_dist[0])
                    action = np.argmax(action_dist == action)

                    action_linear, action_angular = a3c.action_mapper.map_action(action)
                    state1, reward, done, _ = self.env.step(action_linear, action_angular, 10) # @TODO Skipping

                    #self.env.visualize()

                    episode_buffer.append([state, action, reward, state1, done, value[0,0]])
                    episode_value.append(value[0, 0])

                    episode_reward += reward
                    state = state1
                    total_step_count += 1
                    episode_step_count += 1

                    # @TODO Buffer lenght
                    if len(episode_buffer) == 30 and not done and episode_step_count != max_episode_length - 1:
                        v1 = session.run(self.local_AC.critic, feed_dict={self.local_AC.input: [s]})
                        v_l, p_l, e_l, g_n, v_n = self.train(episode_buffer, session, gamma, v1)
                        episode_buffer = []
                        session.run(self.update_local_ops)

                    # @TODO Sinnvoll?
                    if done:
                        break

                self.episode_rewards.append(episode_reward)
                self.episode_lengths.append(episode_step_count)
                self.episode_mean_values.append(np.mean(episode_value))

                # Update the network at the end of the episode
                if len(episode_buffer) != 0:
                    v_l, p_l, e_l, g_n, v_n = self.train(episode_buffer, session, gamma, 0.0)

                # @TODO Auswertung

                # @TODO Increment global
                episode_count += 1

    def train(self, episode_buffer, session, gamma, bootstrap):
        packed = np.array(episode_buffer)
        observations = packed[:, 0]
        actions = packed[:, 1]
        rewards = packed[:, 2]
        next_observations = packed[:, 3]
        values = packed[:, 5]

        self.rewards_plus = np.asarray(rewards.tolist() + [bootstrap])
        discounted_rewards = discount(self.rewards_plus, gamma)[:-1]
        self.value_plus = np.asarray(values.tolist() + [bootstrap])
        advantages = rewards + gamma * self.value_plus[1:] - self.value_plus[:-1]
        advantages = discount(advantages, gamma)

        # Update the global Network
        feed_dict = {self.local_AC.target_v: discounted_rewards,
                     self.local_AC.input: np.vstack(observations),
                     self.local_AC.actions: actions,
                     self.local_AC.advantages: advantages}
        v_l, p_l, e_l, g_n, v_n, _ = session.run([self.local_AC.value_loss,
                                                  self.local_AC.policy_loss,
                                                  self.local_AC.entropy,
                                                  self.local_AC.grad_norms,
                                                  self.local_AC.var_norms,
                                                  self.local_AC.apply_grads],
                                                 feed_dict=feed_dict)

        return v_l / len(packed), p_l / len(packed), e_l / len(packed), g_n, v_n
                



