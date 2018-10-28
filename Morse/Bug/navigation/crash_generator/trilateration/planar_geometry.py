import math


class SinglePointException(BaseException):
    def __init__(self):
        super(self).__init__('By one point infinite lines pass')


class Point2D:
    """ Represents a point in a 2D space """

    def __init__(self, x: float, y: float):
        """ Creates a new point in space """
        self.x = x
        self.y = y

    def distance_from(self, other):
        """ Returns the distance between this point and another """
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def is_close_to(self, other, tolerance):
        """ Returns whether or not this point is close to another, for a given tolerance """
        return self.distance_from(other) < tolerance

    def __add__(self, other):
        """ Adds this point to another """
        return self.__class__(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """ Subtract a point from this one """
        return self.__class__(self.x - other.x, self.y - other.y)

    def __str__(self):
        """ Returns a textual representation of this point """
        return '({0:.2f}, {1:.2f})'.format(self.x, self.y)

    @classmethod
    def from_morse(cls, morse_position):
        """ Returns an instance of this class from Morse position data structure """
        x = morse_position['x']
        y = morse_position['y']
        return cls(x, y)


class Line2D(object):
    """ Represents a straight line in a 2D space """

    def __init__(self, a, b, c):
        """ Creates a new line with equation ax + by + c = 0 """
        self.a = a
        self.b = b
        self.c = c

    def contains_point(self, point: Point2D, tolerance):
        """ Checks whether a point lays on this line or not, with a given tolerance """
        return self.distance_from_point(point) < tolerance

    def __str__(self):
        """ Returns the equation of this line in the space """
        if self.b == 0:
            return "y = {.2f}x + {.2f}".format(-self.a, -self.c)
        else:
            return "{.2f}x + {.2f}y + {.2f} = 0".format(self.a, self.b, self.c)

    @classmethod
    def from_two_points(cls, p1: Point2D, p2: Point2D):
        """ Returns the instance of line passing through the given points """
        if p1.distance_from(p2) == 0:
            raise SinglePointException()
        if p1.x != p2.x:
            m = (p2.y - p1.y) / (p2.x - p1.x)
            q = (-1 * m * p1.x) + p1.y
            return cls.from_explicit_form(m, q)
        else:
            return cls(1, 0, - p1.x)

    @classmethod
    def from_explicit_form(cls, m, q):
        """ Returns a new line with a given slope """
        return cls(-m, 1, -q)

    @classmethod
    def are_collinear(cls, p1, p2, p3, tolerance):
        """ Checks whether three points belong to the same line """
        try:
            return cls.from_two_points(p1, p2).contains_point(p3, tolerance)
        except SinglePointException:
            return True

    def distance_from_point(self, point:Point2D):
        return abs(self.a * point.x + self.b * point.y + self.c) / math.sqrt(self.a ** 2 + self.b ** 2)
