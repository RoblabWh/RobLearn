#! /usr/bin/env python3
import socket
import struct

import sys

import time

import random

import action_getter


class NN_Interface:
    def __init__(self):
        self._is_initialised = False

        self._ip_address_node = "127.0.0.1"
        self._port_node = 55555

        self._ip_address_nn = "127.0.0.1"
        self._port_nn = 55556

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def init(self):
        self._socket.bind((self._ip_address_nn, self._port_nn))

        print("- - - NN_Interface - - -")
        print(" IP_ADDRESS: " + self._ip_address_nn)
        print(" PORT: " + str(self._port_nn))
        print("-> to NN_Interface_Node")
        print(" IP_ADDRESS: " + self._ip_address_node)
        print(" PORT: " + str(self._port_node))
        print("- - -")

        self._is_initialised = True

    def set_address_node(self, ip_address="127.0.0.1", port=55555):
        if self._is_initialised:
            print("Warn[NN_Interface::set_address_nn]: NN_Interface is initialised -> ignore!")
        else:
            self._ip_address_node = ip_address
            self._port_node = port

    def set_address_nn(self, ip_address="127.0.0.1", port=55556):
        if self._is_initialised:
            print("Warn[NN_Interface::set_address_node]: NN_Interface is initialised -> ignore!")
        else:
            self._ip_address_nn = ip_address
            self._port_nn = port

    def send_action(self, linear_velocity, angular_velocity):

        if not self._is_initialised:
            sys.exit("Error[NN_Interface::send_action]: NN_Interface is not initialised -> exit program!")

        data_values = (linear_velocity, angular_velocity)
        s = struct.Struct("! f f")
        data = s.pack(*data_values)

        self._socket.sendto(data, (self._ip_address_node, self._port_node))

    def _read_header(self, header_data):
        s = struct.Struct("! H H H")
        data = s.unpack(header_data)

        datagram_index = int(data[0])
        number_of_packets = int(data[1])
        packet_index = int(data[2])

        return datagram_index, number_of_packets, packet_index


    def _read_observation(self, observation_data):
        size = int(len(observation_data) / 4)
        s = struct.Struct("! " + str(size) + "f")

        data = s.unpack(observation_data)

        return list(data)


    def receive_observation(self):

        if not self._is_initialised:
            sys.exit("Error[NN_Interface::receive_observation]: NN_Interface is not initialised -> exit program!")

        data_received = False

        current_datagram_index = 0
        current_packet_index = 0
        observation = []


        while not data_received:
            data, address = self._socket.recvfrom(1500)

            if address[0] != self._ip_address_node:
                print("Warn[NN_Interface::receive_observation]: Datagram from wrong ip address[" + address[0] + "] -> ignore datagram")
                continue

            datagram_index, number_of_packets, packet_index = self._read_header(data[:6])

            if current_datagram_index != datagram_index:
                current_datagram_index = datagram_index
                current_packet_index = 0

            if current_datagram_index == datagram_index and current_packet_index == packet_index:
                observation = observation + self._read_observation(data[6:])
                current_packet_index += 1
            else:
                observation = []
                continue

            if current_packet_index == number_of_packets:
                data_received = True

        return observation


def main():
    nn_interface = NN_Interface()
    nn_interface.init()

    ag = ActionGetter()

    while True:
        # Get the observation from the laserscan
        observation = nn_interface.receive_observation()

        # TODO: Get the action from observation
        # NN get action
        linear, angular = ag.act(observation)

        # Send the action to the robot
        # nn_interface.send_action(random.uniform(0.0, 0.5), random.uniform(-1, 1))
        nn_interface.send_action(linear, angular)


if __name__ == '__main__':
    main()