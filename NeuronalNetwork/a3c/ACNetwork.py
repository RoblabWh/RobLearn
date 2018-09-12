import tensorflow as tf
import tflearn


class ACNetwork(object):
    def __init__(self, state_size, action_size, scope, trainer):
        with tf.variable_scope(scope):
            self.input = tflearn.layers.input_data(shape=(None, state_size), dtype=tf.float32)
            input = tf.expand_dims(self.input, -1)
            net = input
            net = tflearn.layers.conv_1d(net, 16, 3, padding='same')
            net = tflearn.layers.max_pool_1d(net, 3)
            net = tflearn.layers.conv_1d(net, 16, 2)
            net = tflearn.layers.max_pool_1d(net, 2)
            net = tflearn.layers.fully_connected(net, 64, activation='relu')
            net = tflearn.layers.fully_connected(net, action_size, activation='linear')

            # Actor
            self.actor = tflearn.fully_connected(net, action_size,
                                                 activation=tf.nn.softmax,
                                                 bias_init=None)
            # Critic
            self.critic = tflearn.fully_connected(net, 1, activation=tf.nn.softmax, bias_init=None)

            if scope != 'global':
                self.actions = tf.placeholder(shape=[None], dtype=tf.int32)
                self.actions_onehot = tf.one_hot(self.actions, action_size, dtype=tf.float32)
                self.target_v = tf.placeholder(shape=[None], dtype=tf.float32)
                self.advantages = tf.placeholder(shape=[None], dtype=tf.float32)

                self.responsible_outputs = tf.reduce_sum(self.actor * self.actions_onehot, [1])

                # Loss funcitons
                self.value_loss = 0.5 * tf.reduce_sum(tf.square(self.target_v - tf.reshape(self.critic, [-1])))
                self.entropy = - tf.reduce_sum(self.actor * tf.log(self.actor))
                self.policy_loss = -tf.reduce_sum(tf.log(self.responsible_outputs) * self.advantages)
                self.loss = 0.5 * self.value_loss + self.policy_loss - self.entropy * 0.01

                local_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope)
                self.gradients = tf.gradients(self.loss, local_vars)
                self.var_norms = tf.global_norm(local_vars)
                grads, self.grad_norms = tf.clip_by_global_norm(self.gradients, 40.0)

                global_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'global')
                self.apply_grads = trainer.apply_gradients(zip(grads, global_vars))