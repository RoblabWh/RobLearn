# Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from environment.environment_fitness import Mode


class Config:

    #########################################################################
    # Environment configuration

    # Path of the world
    PATH_TO_WORLD = ["../Simulation2d/world/square"]
    # Use this for multiple Environments in parallel
    #train 1
    #PATH_TO_WORLD = ["../Simulation2d/world/square","../Simulation2d/world/square", "../Simulation2d/world/square"]
    #train_2
    PATH_TO_WORLD = ["../Simulation2d/world/room","../Simulation2d/world/room"]
    #PATH_TO_WORLD = ["../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms"]
    #train_3
    #PATH_TO_WORLD = ["../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms","../Simulation2d/world/four_rooms"]

    #train_4
    NETWORK_DIR =""

    NETWORK_DIR =""

    PATH_TO_WORLD = ["../Simulation2d/world/roblab"]
    #train_5
    #PATH_TO_WORLD = ["../Simulation2d/world/room"]
    # Mode
    MODE=Mode.ALL_RANDOM
    # Terminate the simulation
    TERMINATE_AT_END=False
    # Cluster size of the lidar
    CLUSTER_SIZE=18
    # use observation rotation vector
    USE_OBSERVATION_ROTATION=True
    # Observation rotation vector size
    OBSERVATION_ROTATION_SIZE=16

    # Visualize for training
    VISUALIZE = True
    # Enable to see the trained agent in action
    PLAY_MODE = False
    # Enable to train
    TRAIN_MODELS = True
    # Load old models. Throws if the model doesn't exist
    LOAD_CHECKPOINT =True
    # If 0, the latest checkpoint is loaded
    LOAD_EPISODE = 0

    #########################################################################
    # Number of agents, predictors, trainers and other system settings
    
    # If the dynamic configuration is on, these are the initial values.
    # Number of Agents
    AGENTS = 32#32
    # Number of Predictors
    PREDICTORS = 2#2
    # Number of Trainers
    TRAINERS = 2 #2

    # Device
    DEVICE = 'gpu:0'

    # Enable the dynamic adjustment (+ waiting time to start it)
    DYNAMIC_SETTINGS = False
    DYNAMIC_SETTINGS_STEP_WAIT = 20
    DYNAMIC_SETTINGS_INITIAL_WAIT = 10

    #########################################################################
    # Algorithm parameters

    # Max step Iteration -> if read the environment ist done. 0 for endless.
    MAX_STEP_ITERATION = 1000

    # Discount factor
    DISCOUNT = 0.99
    
    # Tmax
    TIME_MAX = 10 #5
    
    # Reward Clipping
    REWARD_MIN = -1
    REWARD_MAX = 1

    # Max size of the queue
    MAX_QUEUE_SIZE = 200 #100
    PREDICTION_BATCH_SIZE = 256 #128

    # Input of the DNN
    STACKED_FRAMES = 1
    OBSERVATION_SIZE=60

    # Total number of episodes and annealing frequency
    EPISODES = 100000
    ANNEALING_EPISODE_COUNT = 100000

    # Entropy regualrization hyper-parameter
    BETA_START = 0.01
    BETA_END = 0.01

    # Learning rate
    LEARNING_RATE_START = 0.0003
    LEARNING_RATE_END = 0.0003

    # RMSProp parameters
    RMSPROP_DECAY = 0.99
    RMSPROP_MOMENTUM = 0.0
    RMSPROP_EPSILON = 0.1

    # Dual RMSProp - we found that using a single RMSProp for the two cost function works better and faster
    DUAL_RMSPROP = False

    # Gradient clipping
    USE_GRAD_CLIP = False
    GRAD_CLIP_NORM = 40.0 
    # Epsilon (regularize policy lag in GA3C)
    LOG_EPSILON = 1e-6
    # Training min batch size - increasing the batch size increases the stability of the algorithm, but make learning slower
    TRAINING_MIN_BATCH_SIZE = 128 #0
    
    #########################################################################
    # Log and save

    # Enable TensorBoard
    TENSORBOARD = True
    # Update TensorBoard every X training steps
    TENSORBOARD_UPDATE_FREQUENCY = 1000

    # Enable to save models every SAVE_FREQUENCY episodes
    SAVE_MODELS = True
    # Save every SAVE_FREQUENCY episodes
    SAVE_FREQUENCY = 200
    
    # Print stats every PRINT_STATS_FREQUENCY episodes
    PRINT_STATS_FREQUENCY = 1
    # The window to average stats
    STAT_ROLLING_MEAN_WINDOW = 1000

    # Results filename
    RESULTS_FILENAME = 'results.txt'
    # Network checkpoint name
    NETWORK_NAME = 'network60x60'

    #########################################################################
    # More experimental parameters here
    
    # Minimum policy
    MIN_POLICY = 0.0
    # Use log_softmax() instead of log(softmax())
    USE_LOG_SOFTMAX = False
