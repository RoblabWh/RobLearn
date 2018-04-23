#!/usr/bin/python3

import socket
import time
import random

from msg_to_environment_pb2 import MsgToEnvironment
from msg_to_agent_pb2 import MsgToAgent


class EnvironmentGazebo:
    __address = ""
    __port = 0

    __socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self, address: str="127.0.0.1", port:  int=21345):
        self.__address = address
        self.__port = port

    def init(self):
        self.__socket.connect((self.__address, self.__port))

    def step(self, linear_velocity: float, angular_velocity: float):
        self.__send_msg_to_environment(linear_velocity, angular_velocity, False)

        msg = self.__receive_msg_to_agent()

        return msg.observation, msg.reward, msg.done, msg.info

    def __send_msg_to_environment(self, linear_velocity: float, angular_velocity: float, reset):
        msg = MsgToEnvironment()

        msg.reset_environment = reset
        msg.linear_velocity = linear_velocity
        msg.angular_velocity = angular_velocity

        data_msg = msg.SerializeToString()
        data_length = (len(data_msg)).to_bytes(4, 'big')

        data = data_length + data_msg

        self.__socket.sendall(data)

    def __receive_msg_to_agent(self):
        data_length = self.__socket.recv(4)

        msg_length = int.from_bytes(data_length, 'big')

        data_msg = self.__socket.recv(msg_length)

        msg = MsgToAgent()

        msg.ParseFromString(data_msg)

        return msg







def main():
    env = EnvironmentGazebo()

    env.init()

    while True:
        time.sleep(0.0001)
        next_state, reward, done, _ = env.step(random.uniform(1, 2), random.uniform(-1.0, 1.0))


if __name__ == "__main__":
    main()
