#!/usr/bin/env python3

import math
from .environment_node_data import NodeData, Mode


class FitnessData:
    """
    Class for calculating the fitness function reward from the current simulation state. Possible reward calculation
    are: travelled distance, distance to the target node, angle to the target node etc.
    """

    def __init__(self):
        """
        Constructor of the FitnessData class.
        """
        self._node_data = NodeData()

        self._terminate_at_end = True
        self.__target_value = 20
        self.__segment_size = 2
        self.__frame_width = 12
        self.__frame_height = 12
        self.__robot_segments = []
        self._robot_x_last = 0.0
        self._robot_y_last = 0.0
        self.__robot_last_seg = 0
        self._robot_orientation_last = 0.0

    def get_end_node(self):
        """
        Get the current selected end Node.
        :return:
        """
        return self._node_data.get_node_end()

    def init(self, filename="") -> bool:
        """
        Read the node file and initialized the node data.
        :param filename:
        :return:
        """
        return self._node_data.read_node_file(filename)

    def set_mode(self, mode: Mode, terminate_at_end=True):
        """
        Set the mode for the simulation. More information in environment.py.
        :param mode: Simulation mode.
        :param terminate_at_end: Done when the target node is reached.
        :return:
        """
        self._node_data.set_mode(mode)
        self._terminate_at_end = terminate_at_end

    def _distance_start_to_end(self) -> float:
        """
        Calculate the distance between the start node and end node.
        :return: Distance between start and end node.
        """
        return self._distance(
            self._node_data.get_node_start().x(),
            self._node_data.get_node_start().y(),
            self._node_data.get_node_end().x(),
            self._node_data.get_node_end().y())

    def _distance_robot_to_end(self, robot_x: float, robot_y: float) -> float:
        """
        Calculate the distance between the robot position to the end node.
        :param robot_x: Robot position x.
        :param robot_y: Robot position y.
        :return: Distance between robot and end node.
        """
        return self._distance(
            robot_x,
            robot_y,
            self._node_data.get_node_end().x(),
            self._node_data.get_node_end().y())

    def _distance_between_last_step(self, robot_x: float, robot_y: float) -> float:
        """
        Calculate the distance from the last robot position to the current robot position.
        :param robot_x: Robot position x.
        :param robot_y: Robot position y.
        :return: Distance from last and current robot position.
        """
        return self._distance(
            robot_x,
            robot_y,
            self._robot_x_last,
            self._robot_y_last)

    def _orientation_robot_to_end(self, robot_x: float, robot_y: float) -> float:
        """
        Calculate the orientation from the robot position to the end node.
        :param robot_x: Robot position x.
        :param robot_y: Robot position y.
        :return: Orientation from robot position to end node.
        """
        x = self._node_data.get_node_end().x() - robot_x
        y = self._node_data.get_node_end().y() - robot_y
        return math.atan2(y, x)

    def angle_difference_from_robot_to_end(self, robot_x: float, robot_y: float, robot_orientation: float) -> float:
        """
        Calculate the angle difference from robot to end node.
        :param robot_x: Robot position x.
        :param robot_y: Robot position y.
        :return: Angle difference robot to end node.
        """
        return self._difference_two_angles(robot_orientation, self._orientation_robot_to_end(robot_x, robot_y))

    @staticmethod
    def _difference_two_angles(angle1: float, angle2: float) -> float:
        """
        Calculate the difference between to angle.
        :param angle1: First angle.
        :param angle2: Second angle.
        :return: Difference between to angle.
        """
        diff = (angle1 - angle2) % (math.pi * 2)
        if diff >= math.pi:
            diff -= math.pi * 2
        return diff

    @staticmethod
    def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Calculate the euler distance from to point.
        :param x1: First point x.
        :param y1: First point y.
        :param x2: Second point x.
        :param y2: Second point y.
        :return: Euler distnace from to points.
        """
        x = x1 - x2
        y = y1 - y2
        return math.sqrt(x*x + y*y)

    def reset(self):
        """
        Reset call from the environment to select new node.
        :return:
        """
        self._node_data.new_node_selection()

    def calculate_reward(self, robot_x: float, robot_y: float, robot_orientation: float, env_done: bool):
        """
        Calculate the reward from one step.
        :param robot_x: Robot position x.
        :param robot_y: Robot position y.
        :param robot_orientation: Robot orientation angle.
        :param env_done: Done when the robot collided with the robot.
        :return: Reward, done.
        """
        done = False
        reward = -1


        robot_actu_seg = self.__get_segment_of_point(robot_x , robot_y)
        target_act_seg = self.__get_segment_of_point(self._node_data.get_node_end().x() , self._node_data.get_node_end().y())
        start_seg = self.__get_segment_of_point(self._node_data.get_node_start().x() , self._node_data.get_node_start().y())
        if not env_done:
            self.__robot_segments.append(robot_actu_seg)
        
        #distance_start_to_end = self.__distance_start_to_end()
        distance_robot_to_end = self._distance_robot_to_end(robot_x, robot_y)
        distance_robot_to_start = self._distance_robot_to_start(robot_x, robot_y)
        distance_between_last_step = self._distance_between_last_step(robot_x, robot_y)
        distance_robot_to_end_last = self._distance_robot_to_end(self._robot_x_last, self._robot_y_last)
        distance_robot_to_end_diff = distance_robot_to_end_last - distance_robot_to_end;

        #reward = self.__map_segment_to_value(robot_actu_seg,target_act_seg)
        if self.__robot_last_seg ==  robot_actu_seg:
           reward -= 2

        if distance_robot_to_end_diff < 0:
            distance_robot_to_end_diff *= 2
        else:
            distance_robot_to_end_diff *= 1.5
        
        reward += distance_robot_to_end_diff
        
        #reward = distance_between_last_step + (1 - distance_robot_to_end / distance_start_to_end) + distance_between_last_step

        #reward = 0
        # reward += 10 * max((10 - self._distance_robot_to_end(robot_x, robot_y)) / 10, 0)
        # reward += distance_robot_to_start
        # reward += distance_between_last_step
        # reward -= distance_robot_to_end

        reward += (math.pi - math.fabs(self._difference_two_angles(robot_orientation, self._orientation_robot_to_end(robot_x, robot_y)))) / math.pi
        reward += 1/(self.__map_segment_to_value(robot_actu_seg,target_act_seg)+1)
        #print("rewarddddd:  ", reward)
        #reward += max((5 - self._distance_robot_to_end(robot_x, robot_y)) / 5, 0)
        #reward = 0

        if env_done:
            reward = -100 #- distance_robot_to_end / distance_start_to_end * 100
            self.__robot_segments = []
            done = True
        elif distance_robot_to_end < self._node_data.get_node_end().radius():
            reward = 100
            done = self._handle_terminate_at_end()
        # else:
        # #     # reward = 1
        # #     reward = -1
        # #     # reward = 0

        self._robot_x_last = robot_x
        self._robot_y_last = robot_y
        self._robot_orientation_last = robot_orientation
        self.__robot_last_seg = robot_actu_seg

        return reward, done


    def __get_segment_of_point(self, x: float , y: float):
        vertical_segment_num = self.__frame_width/ self.__segment_size 
        actu_segment = math.floor(y / self.__segment_size) * vertical_segment_num + math.floor(x / self.__segment_size)
        return actu_segment

     
    def __get_distance_of_segments(self, segment1:float, segment2:float):
        vertical_segment_num = self.__frame_width/ self.__segment_size 
        seg1_row = math.floor(segment1/vertical_segment_num) + 1
        seg1_col = (segment1 - (math.floor(segment1/vertical_segment_num)*vertical_segment_num))+1
        seg2_row = math.floor(segment2/vertical_segment_num) + 1
        seg2_col = (segment1 - (math.floor(segment1/vertical_segment_num)*vertical_segment_num))+1
        distance = max((math.fabs(seg2_row-seg1_row)),(math.fabs(seg2_col-seg1_col)))
        return distance

    def __map_segment_to_value(self, segment:float , target_segment:float):
        distance = self.__get_distance_of_segments(segment, target_segment)

       # value = self.__target_value / math.pow(2,distance) 
        return distance


    def _distance_robot_to_start(self, robot_x: float, robot_y: float) -> float:
        """
        Calculate the distance between the robot position to the end node.
        :param robot_x: Robot position x.
        :param robot_y: Robot position y.
        :return: Distance between robot and end node.
        """
        return self._distance(
            robot_x,
            robot_y,
            self._node_data.get_node_start().x(),
            self._node_data.get_node_start().y())

    def get_robot_start(self):
        """
        Get the start node for the start position for the robot.
        :return: Start node.
        """
        return self._node_data.generate_robot_start_position()

    def _handle_terminate_at_end(self):
        """
        Select a new target node when the simulation souldn't terminate when reaching the target node.
        :return:
        """
        if not self._terminate_at_end:
            self._node_data.new_end_node()

        return self._terminate_at_end
