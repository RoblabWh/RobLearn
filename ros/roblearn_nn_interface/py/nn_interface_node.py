#! /usr/bin/env python
import socket
import threading
import struct
import sys

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy


class NNInterfaceNode:
    def __init__(self):
        self._is_initialised = False
        self._nn_control = False

        self._ip_address_node = "127.0.0.1"
        self._port_node = 55555

        self._ip_address_nn = "127.0.0.1"
        self._port_nn = 55556

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._datagram_index = 0
        self._datagram_size_slice = 256 # * 4 byte size of the packet

        self._publisher_cmd_vel = None
        self._laserscan_counter = 0
        self._laserscan_counter_max = 20

        self._joy_linear = 0
        self._joy_angular = 0



    def init(self):
        # init socket
        self._socket.bind((self._ip_address_node, self._port_node))
        self._socket.settimeout(1)

        # init ros
        rospy.init_node("nn_interface_node")
        self._publisher_cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._callback_laserscan)
        rospy.Subscriber("/joy", Joy, self._callback_joy)

        print("- - - NN_Interface_Node - - -")
        print(" IP_ADDRESS: " + self._ip_address_node)
        print(" PORT: " + str(self._port_node))
        print("-> to NN_Interface")
        print(" IP_ADDRESS: " + self._ip_address_nn)
        print(" PORT: " + str(self._port_nn))
        print("- - -")

        self._is_initialised = True


    def set_address_node(self, ip_address="127.0.0.1", port=55555):
        if self._is_initialised:
            print("Warn[NN_Interface_Node::set_address_node]: NN_Interface_Node is initialised -> ignore!")
        else:
            self._ip_address_node = ip_address
            self._port_node = port


    def set_address_nn(self, ip_address="127.0.0.1", port=55556):
        if self._is_initialised:
            print("Warn[NN_Interface_Node::set_address_nn]: NN_Interface_Node is initialised -> ignore!")
        else:
            self._ip_address_nn = ip_address
            self._port_nn = port


    def _callback_laserscan(self, data):
        if self._laserscan_counter < self._laserscan_counter_max:
            self._laserscan_counter += 1
        else:
            range_max = 20.0
            range_min = data.range_min
            observation = []

            for range in data.ranges:
                if range < range_min:
                    range = 0.0
                elif range_max < range:
                    range = 1.0
                else:
                    range = range / range_max
                observation.append(float(range))

            self._send_observation(observation)

            self._laserscan_counter = 0

    def _callback_joy(self, data):
        if data.buttons[1]:
            self._nn_control = True
        elif data.buttons[7]:
            self._joy_linear = data.axes[1]
            self._joy_angular = 2.5 * data.axes[2]
            self._nn_control = False
        else:
            self._joy_linear = 0.5 * data.axes[1]
            self._joy_angular = 1.5 * data.axes[2]
            self._nn_control = False

    def _publish_twist(self, linear_velocity, angular_velocity):
        msg_twist = Twist()

        msg_twist.linear.x = float(linear_velocity)
        msg_twist.linear.y = 0
        msg_twist.linear.z = 0
        msg_twist.angular.x = 0
        msg_twist.angular.y = 0
        msg_twist.angular.z = float(angular_velocity)

        self._publisher_cmd_vel.publish(msg_twist)


    def _send_observation(self, observation):
        index_slice = 0
        size_data = len(observation)
        number_of_packets = size_data / self._datagram_size_slice

        if size_data % self._datagram_size_slice!= 0:
            number_of_packets += 1

        for index_packet in range(number_of_packets):
            size_slice = self._datagram_size_slice

            if size_data < size_slice * (index_packet + 1) :
                size_slice = size_slice * (index_packet + 1) - size_data

            observation_slice = observation[index_slice: (index_slice + size_slice)]

            data_values = (self._datagram_index, number_of_packets, index_packet) + tuple(observation_slice)
            s = struct.Struct("! H H H " + str(len(observation_slice)) + "f")

            data = s.pack(*data_values)

            self._socket.sendto(data, (self._ip_address_nn, self._port_nn))

            index_slice += self._datagram_size_slice

        # begin from zero when a short overflow is happen
        self._datagram_index = (self._datagram_index + 1) % 65535



    def _worker_udp_communication(self):
        while not rospy.is_shutdown():
            if self._nn_control:
                try:
                    data, address = self._socket.recvfrom(8)

                    if address[0] != self._ip_address_nn:
                        print("Warn[NN_Interface_Node::receive_observation]: Datagram from wrong ip address[" + address[
                            0] + "] -> ignore datagram")
                        continue

                    s = struct.Struct("! f f")
                    data_velocity = s.unpack(data)

                    self._publish_twist(data_velocity[0], data_velocity[1])

                except socket.timeout:
                    # stop robot if a timeout exception is raised
                    self._publish_twist(0.0, 0.0)
            else:
                self._publish_twist(self._joy_linear, self._joy_angular)
                rospy.sleep(0.01)


    def run(self):
        if not self._is_initialised:
            sys.exit("Error[NN_Interface_Node::run]: NN_Interface_Node is not initialised -> exit program!")

        t = threading.Thread(target=self._worker_udp_communication())
        t.start()

        rospy.spin()

        t.join()


def main():
    nn_interface_node = NNInterfaceNode()
    
    nn_interface_node.init()
    nn_interface_node.run()





if __name__ == '__main__':
    main()
