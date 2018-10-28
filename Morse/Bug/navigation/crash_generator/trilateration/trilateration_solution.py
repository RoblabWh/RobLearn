import numpy as np
from crash_generator.trilateration.planar_geometry import Line2D, Point2D


class CollinearityException(BaseException):
    def __init__(self):
        super(CollinearityException, self).__init__('Points are collinear.')


class TrilaterationSolver:
    def __init__(self, tolerance=0.2):
        """ Initializes the solver with a given tolerance """
        self.tolerance = tolerance

    def solve(self, p1: Point2D, r1, p2: Point2D, r2, p3: Point2D, r3):
        """ Trilaterate a target position given three points and their distance from it """
        P1 = np.array([p1.x, p1.y])
        P2 = np.array([p2.x, p2.y])
        P3 = np.array([p3.x, p3.y])

        ex = (P2 - P1) / np.linalg.norm((P2 - P1))
        i = np.dot(ex, (P3 - P1))
        ey = (P3 - P1 - i * ex) / np.linalg.norm((P3 - P1 - i * ex))
        d = np.linalg.norm((P2 - P1))
        j = np.dot(ey, (P3 - P1))
        x = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
        y = (r1 ** 2 - r3 ** 2 + i ** 2 + j ** 2) / (2 * j) - (i * x) / j
        target = P1 + x * ex + y * ey

        return Point2D(target[0], target[1])
