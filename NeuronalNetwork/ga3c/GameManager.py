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

from action_mapper import map_action
from environment.environment import Environment

from .Config import Config

import time

class GameManager:
    def __init__(self, id):

        self.visualize = False

        if Config.VISUALIZE and id == 0:
            self.visualize = True
        elif Config.PLAY_MODE:
            self.visualize = True

        self.env = Environment(Config.PATH_TO_WORLD)

        self.env.set_mode(Config.MODE, Config.TERMINATE_AT_END)
        self.env.set_observation_rotation_size(Config.OBSERVATION_ROTATION_SIZE)
        self.env.use_observation_rotation_size(Config.USE_OBSERVATION_ROTATION)
        self.env.set_cluster_size(Config.CLUSTER_SIZE)

        self.reset()

    def reset(self):
        observation, _, _, _ = self.env.reset()
        return observation

    def step(self, action):
        self._update_display()
        if action is None:
            observation, reward, done, info = self.env.step(0, 0, 20)
            reward = 0
            done = False
        else:
            linear, angular = map_action(action)
            observation, reward, done, info = self.env.step(linear, angular,20)
        return observation, reward, done, info

    def _update_display(self):
        if self.visualize:
            self.env.visualize()

    def observation_size(self):
        return self.env.observation_size()