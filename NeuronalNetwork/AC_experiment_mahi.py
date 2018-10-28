
import math
import numpy as np 
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input
from keras.layers.merge import Add, Multiply
from keras.optimizers import Adam
from environment.environment import Environment
import keras.backend as K

import tensorflow as tf

import random
from collections import deque

# determines how to assign values to each state, i.e. takes the state
# and action (two-input model) and determines the corresponding value
class ActorCritic:
	def __init__(self, env, sess):
		self.env  = env
		self.sess = sess

		self.learning_rate = 0.005
		self.epsilon = 1.0
		self.epsilon_decay = .995
		self.gamma = .95
		self.tau   = .125

		# ===================================================================== #
		#                               Actor Model                             #
		# Chain rule: find the gradient of chaging the actor network params in  #
		# getting closest to the final value network predictions, i.e. de/dA    #
		# Calculate de/dA as = de/dC * dC/dA, where e is error, C critic, A act #
		# ===================================================================== #

		self.memory = deque(maxlen=5000)
		self.actor_state_input, self.actor_model = self.create_actor_model()
		_, self.target_actor_model = self.create_actor_model()
		action_size = 5
		self.actor_critic_grad = tf.placeholder(tf.float32, 
			[None, action_size]) # where we will feed de/dC (from critic)
		
		actor_model_weights = self.actor_model.trainable_weights
		self.actor_grads = tf.gradients(self.actor_model.output, 
			actor_model_weights, -self.actor_critic_grad) # dC/dA (from actor)
		grads = zip(self.actor_grads, actor_model_weights)
		self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(grads)

		# ===================================================================== #
		#                              Critic Model                             #
		# ===================================================================== #		

		self.critic_state_input, self.critic_action_input, \
			self.critic_model = self.create_critic_model()
		_, _, self.target_critic_model = self.create_critic_model()

		self.critic_grads = tf.gradients(self.critic_model.output, 
			self.critic_action_input) # where we calcaulte de/dC for feeding above
		
		# Initialize for later gradient calculations
		self.sess.run(tf.initialize_all_variables())

	# ========================================================================= #
	#                              Model Definitions                            #
	# ========================================================================= #

	def create_actor_model(self):
		state_input = Input(shape=(self.env.observation_size(),))
		h1 = Dense(24, activation='linear')(state_input)
		h2 = Dense(48, activation='linear')(h1)
		h3 = Dense(24, activation='linear')(h2)
		action_size = 5
		output = Dense(action_size, activation='linear')(h3)
		
		model = Model(input=state_input, output=output)
		adam  = Adam(lr=0.001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, model

	def create_critic_model(self):
		state_input = Input(shape=(self.env.observation_size(),))
		state_h1 = Dense(24, activation='linear')(state_input)
		state_h2 = Dense(48)(state_h1)
		action_size = 5
		action_input = Input(shape=(action_size,))
		action_h1    = Dense(48)(action_input)
		
		merged    = Add()([state_h2, action_h1])
		merged_h1 = Dense(24, activation='linear')(merged)
		output = Dense(1, activation='linear')(merged_h1)
		model  = Model(input=[state_input,action_input], output=output)
		
		adam  = Adam(lr=0.001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, action_input, model

	# ========================================================================= #
	#                               Model Training                              #
	# ========================================================================= #

	def remember(self, cur_state, action, reward, new_state, done):
		self.memory.append([cur_state, action, reward, new_state, done])

	def _train_actor(self, samples):
		for sample in samples:
			cur_state, action, reward, new_state, _ = sample
			predicted_action = self.actor_model.predict(cur_state)
			grads = self.sess.run(self.critic_grads, feed_dict={
				self.critic_state_input:  cur_state,
				self.critic_action_input: predicted_action
			})[0]
			self.sess.run(self.optimize, feed_dict={
				self.actor_state_input: cur_state,
				self.actor_critic_grad: grads
			})
            
	def _train_critic(self, samples):
		for sample in samples:
			cur_state, action, reward, new_state, done = sample
			if not done:
				target_action = self.target_actor_model.predict(new_state)
				future_reward = self.target_critic_model.predict(
					[new_state, target_action])[0][0]
				reward += self.gamma * future_reward
			self.critic_model.fit([cur_state, np.reshape(action,(1,5))], np.reshape(reward,(1,1)), verbose=0)

	def train(self):
		batch_size =32
		if len(self.memory) < batch_size:
			return

		rewards = []
		samples = random.sample(self.memory, batch_size)
		self._train_critic(samples)
		self._train_actor(samples)

	# ========================================================================= #
	#                         Target Model Updating                             #
	# ========================================================================= #

	def _update_actor_target(self):
		actor_model_weights  = self.actor_model.get_weights()
		actor_target_weights = self.target_critic_model.get_weights()
		
		for i in range(len(actor_target_weights)):
			actor_target_weights[i] = actor_model_weights[i]
		self.target_critic_model.set_weights(actor_target_weights)

	def _update_critic_target(self):
		critic_model_weights  = self.critic_model.get_weights()
		critic_target_weights = self.critic_target_model.get_weights()
		
		for i in range(len(critic_target_weights)):
			critic_target_weights[i] = critic_model_weights[i]
		self.critic_target_model.set_weights(critic_target_weights)		

	def update_target(self):
		self._update_actor_target()
		self._update_critic_target()

	# ========================================================================= #
	#                              Model Predictions                            #
	# ========================================================================= #

	def act(self, cur_state):
		self.epsilon *= self.epsilon_decay
		if np.random.random() < self.epsilon:
			#random_actions = np.array([random.uniform(0,1),random.uniform(0,1)])
			return self.actor_model.predict(cur_state)
		return self.actor_model.predict(cur_state)

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


def main():
	sess = tf.Session()
	K.set_session(sess)
	env = Environment("test")
	actor_critic = ActorCritic(env, sess)
	done = False
	num_trials = 10000
	trial_len  = 500

	steps = []
	state_size = env.observation_size()
	for trial in range(num_trials):
		reward_sum = 0
		cur_state,_,_,_ = env.reset()
		cur_state = np.reshape(cur_state, [1,state_size])
		
		for step in range(trial_len):
			action = actor_critic.act(cur_state)
			action2 = np.argmax(action[0])
			linear , angular = convert_action(action2)
			#print("action", action)
			#linear = action[0][0]
			#linear = np.array([linear])
			#linear = float(linear[0])
			#linear = (0.8/math.pi)*math.atan((linear-0.5))+0.45
			#2/pi*atan(50*(x-0.5))
			#print("linear", linear)	
			#angular =action[0][1]# 0.77
			#angular = np.array([angular])
			#angular = float(angular[0])
			#1/pi*atan(15*(x-0.5))+0.5
			#angular = (2/math.pi)*math.atan((angular - 0.5))
			new_state, reward, done, _ = env.step(linear, angular,20)
			new_state = np.reshape(new_state, [1, state_size])
			reward_sum = reward_sum + reward
			print("reward---", reward_sum)
			actor_critic.remember(cur_state, action, reward, new_state, done)
			cur_state = new_state
			env.visualize()
			if done:
				break
		actor_critic.train()


if __name__ == "__main__":
	main()
