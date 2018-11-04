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

import sys

if sys.version_info >= (3, 0):
    from queue import Queue
else:
    from Queue import Queue

import numpy as np

from .Config import Config
from .GameManager import GameManager
from action_mapper import ACTION_SIZE


class Environment:
    def __init__(self, id=-1):
        self.game = GameManager(id)
        self.nb_frames = Config.STACKED_FRAMES
        self.frame_q = Queue(maxsize=self.nb_frames)
        self.previous_state = None
        self.current_state = None
        self.total_reward = 0

        self.reset()

    def _get_current_state(self):
        if not self.frame_q.full():
            return None  # frame queue is not full yet.
        _maps = np.array([i[0][0] for i in self.frame_q.queue])
        _rotations = np.array([i[1][0] for i in self.frame_q.queue])
        _maps = np.reshape(_maps, (Config.OBSERVATION_SIZE, 100,self.nb_frames))
        _rotations = np.reshape(_rotations, (Config.OBSERVATION_ROTATION_SIZE, self.nb_frames))
        # x_ = np.transpose(x_, [1, 2, 0])  # move channels
        # x_ = np.transpose(x_, [1, 0])  # move channels
        return np.array([_maps, _rotations])

    def _update_frame_q(self, frame):
        if self.frame_q.full():
            self.frame_q.get()
        self.frame_q.put(frame)

    def get_num_actions(self):
        return ACTION_SIZE

    def reset(self):
        self.total_reward = 0
        self.frame_q.queue.clear()
        self._update_frame_q(self.game.reset())
        self.previous_state = self.current_state = None

    def step(self, action):
        observation, reward, done, _ = self.game.step(action)

        self.total_reward += reward
        self._update_frame_q(observation)

        self.previous_state = self.current_state
        self.current_state = self._get_current_state()
        return reward, done
