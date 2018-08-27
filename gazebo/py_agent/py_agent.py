#!/usr/bin/python3

import socket
import time
import random

from msg_to_environment_pb2 import MsgToEnvironment
from msg_to_agent_pb2 import MsgToAgent


class EnvironmentGazebo:
    """
    Class for the communication to the gz enviromnent.
    """
    __address = ""
    __port = 0

    __timeout = 2.5

    __socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self, address: str = "127.0.0.1", port: int = 21345):
        """
        Constructor of the EviromnentGazebo.
        :param address: Address to gz enviromnent.
        :param port: Port to gz enviromnent.
        """
        self.__address = address
        self.__port = port

    def init(self):
        """
        Initialize the communication to the gz environment.
        :return:
        """
        self.__socket.settimeout(self.__timeout)
        self.__socket.connect((self.__address, self.__port))

        print("Connect to environment.")

    def __reconnecting(self):
        """
        Reconnecting to the gz environment.
        :return:
        """
        self.__socket.close()

        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.settimeout(self.__timeout)

        time.sleep(0.5)

        self.init()

    def step(self, linear_velocity: float, angular_velocity: float):
        """
        Send the action to the gz environment.
        :param linear_velocity: Linear velocity of the turtlebot.
        :param angular_velocity: Angular velocity of the turtlebot.
        :return:
        """
        success = False

        while not success:
            try:

                self.__send_msg_to_environment(linear_velocity, angular_velocity, False)

                msg = self.__receive_msg_to_agent()

                return msg.observation, msg.reward, msg.done, msg.info
            # Reconnect to the gz environment if no message comes back. Avoiding deadlock if the simulation is to fast.
            except socket.timeout:
                print("[EnvironmentGazebo][step]: TimeoutException. Try to reconnect.")

                self.__reconnecting()

    def __send_msg_to_environment(self, linear_velocity: float, angular_velocity: float, reset):
        """
        Create action message for the gz environment and send it.
        :param linear_velocity: Linear velocity of the turtlebot.
        :param angular_velocity: Angular velocity of the turtlebot.
        :param reset: Command to reset the gazebo simulation.
        :return:
        """
        msg = MsgToEnvironment()

        msg.reset_environment = reset
        msg.linear_velocity = linear_velocity
        msg.angular_velocity = angular_velocity

        data_msg = msg.SerializeToString()
        data_length = (len(data_msg)).to_bytes(4, 'big')

        data = data_length + data_msg

        self.__socket.sendall(data)

    def __receive_msg_to_agent(self):
        """
        Wait for incoming message of the gz environment.
        :return:
        """

        # read message length
        data_length = self.__socket.recv(4)

        msg_length = int.from_bytes(data_length, 'big')

        # read message content
        data_msg = self.__socket.recv(msg_length)

        msg = MsgToAgent()

        # Parse message content to the protobuffer message.
        msg.ParseFromString(data_msg)

        return msg


def main():
    """
    Main function for testing the gz environment communication.
    :return:
    """
    env = EnvironmentGazebo()

    env.init()

    while True:
        next_state, reward, done, _ = env.step(random.uniform(1, 2), random.uniform(-1.0, 1.0))


if __name__ == "__main__":
    main()
