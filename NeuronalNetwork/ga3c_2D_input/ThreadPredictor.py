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

from threading import Thread

import numpy as np

from .Config import Config


class ThreadPredictor(Thread):
    def __init__(self, server, id):
        super(ThreadPredictor, self).__init__()
        self.setDaemon(True)

        self.id = id
        self.server = server
        self.exit_flag = False

    def run(self):
        ids = np.zeros(Config.PREDICTION_BATCH_SIZE, dtype=np.uint16)
        states = np.zeros(
            (Config.PREDICTION_BATCH_SIZE, Config.OBSERVATION_SIZE, 100,  Config.STACKED_FRAMES),
            dtype=np.float32)
        rotations = np.zeros(
            (Config.PREDICTION_BATCH_SIZE, Config.OBSERVATION_ROTATION_SIZE, Config.STACKED_FRAMES),
            dtype=np.float32)

        while not self.exit_flag:
            i, x = self.server.prediction_q.get()
            ids[0], states[0], rotations[0] = i, x[0], x[1]

            size = 1
            while size < Config.PREDICTION_BATCH_SIZE and not self.server.prediction_q.empty():
                i, x = self.server.prediction_q.get()
                ids[size], states[size], rotations[size] = i, x[0], x[1]
                size += 1

            state_batch = states[:size]
            rotation_batch = rotations[:size]
            p, v = self.server.model.predict_p_and_v([state_batch, rotation_batch])

            for i in range(size):
                if ids[i] < len(self.server.agents):
                    self.server.agents[ids[i]].wait_q.put((p[i], v[i]))
