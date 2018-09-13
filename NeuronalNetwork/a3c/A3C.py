import multiprocessing
import threading

import tensorflow as tf

from a3c import ACNetwork, action_mapper, Worker
from environment.environment import Environment

WORLD_NAME = 'test'
NUM_WORKERS = 4

MAX_EPISODE_LENGTH = 300
GAMMA = .99 # Discount Rate


def main():
    env = Environment(WORLD_NAME)
    env.set_cluster_size(30)
    env.use_observation_rotation_size(False)

    with tf.device('/cpu:0'):
        master_network = ACNetwork.ACNetwork(env.observation_size(), action_mapper.ACTION_SIZE, 'global', None)
        num_workers = multiprocessing.cpu_count() if NUM_WORKERS == 0 else NUM_WORKERS
        trainer = tf.train.AdamOptimizer(learning_rate=1e-4)

        workers = []
        for i in range(num_workers):
            workers.append(Worker.Worker(WORLD_NAME, i, env.observation_size(), action_mapper.ACTION_SIZE, trainer, None, None))

    with tf.Session() as session:
        coordinator = tf.train.Coordinator()

        # @TODO Load the model
        session.run(tf.global_variables_initializer())

        worker_threads = []
        for worker in workers:
            worker_work = lambda: worker.work(MAX_EPISODE_LENGTH, GAMMA, session, coordinator, None)
            thread = threading.Thread(target=worker_work)
            thread.start()
            worker_threads.append(thread)

        coordinator.join(worker_threads)


if __name__ == '__main__':
    main()