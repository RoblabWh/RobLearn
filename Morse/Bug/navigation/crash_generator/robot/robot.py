import math
from pymorse import pymorse

from crash_generator import configuration
from crash_generator.trilateration.planar_geometry import Point2D


class Robot:
    """ A wrapper class for Morse robot data structure """

    SENSORS_RANGE = 2

    def __init__(self, morse_robot: pymorse.Robot, simulation: pymorse.Morse):
        """ Creates a new class connected to a Morse robot """
        self._morse_object = morse_robot
        self._simulation = simulation

    @property
    def position(self):
        """ Returns the current position of this robot """
        return Point2D.from_morse(self._morse_object.pose.get())

    @property
    def distance_to_target(self):
        """ Returns the current distance to the goal position """
        distance = self._morse_object.prox.get()['near_robots'].get('GOAL')
        if distance is None:
            distance = math.inf
        return distance

    @property
    def orientation(self):
        """ Returns the angle of orientation, laying in (-pi, +pi) """
        return self._morse_object.pose.get()['yaw']

    @property
    def free_space_ahead(self):
        """ Returns the available space ahead, as read from infrared sensors"""
        return min(self._morse_object.ir1.get()['range_list'])

    @property
    def free_space_left(self):
        """ Returns the available space at left, as read from infrared sensors"""
        return min(self._morse_object.ir2.get()['range_list'])

    @property
    def free_space_right(self):
        """ Returns the available space at right, as read from infrared sensors"""
        return min(self._morse_object.ir3.get()['range_list'])

    @property
    def free_space_back(self):
        """ Returns the available space at the back of the robot, as read from infrared sensors"""
        return min(self._morse_object.ir4.get()['range_list'])

    @property
    def risk_of_collision(self):
        return min([self.free_space_ahead, self.free_space_left,
                    self.free_space_right]) < configuration.RISK_OF_COLLISION_DISTANCE

    def set_velocity(self, linear, angular):
        """ Sets both linear and angular velocity of this robot """
        self._morse_object.motion.publish({'v': linear, 'w': angular})
        return self

    def rotate_of(self, radiants, velocity):
        """ Rotates the robot of a given angle with a certain angular velocity """
        self.stop()
        delay = abs(radiants / velocity)
        velocity = velocity if radiants > 0 else -velocity
        self.set_velocity(0, velocity).stop_after(delay)
        return self

    def rotate_to(self, radiants, velocity):
        """ Rotates the robot to a given orientation with a certain angular velocity """
        self.rotate_of(radiants - self.orientation, velocity)
        return self

    def stop(self):
        """ Immediately stops the robot """
        self.set_velocity(0, 0)
        return self

    def stop_after(self, seconds):
        """ Stops the robot after a certain delay in seconds.\n Delay is not asynchronous """
        self.sleep(seconds).stop()
        return self

    def sleep(self, seconds):
        """Does nothing for a given amount of time"""
        self._simulation.sleep(seconds)
        return self
