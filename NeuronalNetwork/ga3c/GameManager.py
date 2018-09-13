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

from environment.environment_fitness import Mode

class GameManager:
    def __init__(self, path_to_world, mode=Mode.ALL_COMBINATION, terminate=True, visualize=False, cluster_size=1,use_observation_rotation=True, observation_rotation_size=64):
        self.visualize = visualize

        self.env = Environment(path_to_world)

        self.env.set_mode(mode=mode, terminate_at_end=terminate)
        self.env.set_observation_rotation_size(observation_rotation_size)
        self.env.use_observation_rotation_size(use_observation_rotation)
        self.env.set_cluster_size(cluster_size)

        self.reset()

    def reset(self):
        observation, _, _, _ = self.env.reset()
        return observation

    def step(self, action):
        self._update_display()
        linear, angular = map_action(action)
        observation, reward, done, info = self.env.step(linear, angular,20)
        return observation, reward, done, info

    def _update_display(self):
        if self.visualize:
            self.env.visualize()

    def observation_size(self):
        return self.env.observation_size()