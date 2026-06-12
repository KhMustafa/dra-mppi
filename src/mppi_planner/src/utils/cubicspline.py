import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize


class CubicSpline2D:
    """
    2D Cubic Spline class
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline(self.s, x, bc_type='natural')
        self.sy = CubicSpline(self.s, y, bc_type='natural')

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx(s)
        y = self.sy(s)
        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx(s, 1)
        ddx = self.sx(s, 2)
        dy = self.sy(s, 1)
        ddy = self.sy(s, 2)
        k = (dx * ddy - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        return k

    def calc_curvature_d(self, s):
        """
        calc first derivative of curvature dk/ds
        """
        dx = self.sx(s, 1)
        ddx = self.sx(s, 2)
        dddx = self.sx(s, 3)
        dy = self.sy(s, 1)
        ddy = self.sy(s, 2)
        dddy = self.sy(s, 3)
        # ---------------------------------
        # k(s) = Z(s) / N(s) (nominator/denominator)
        # dk/ds = (dZ/ds N - Z dN/ds) / (N^2)
        # ---------------------------------
        Z = dx * ddy - ddy * dy
        Z_d = dx * dddy - dddx * dy
        N = (dx ** 2 + dy ** 2) ** (3 / 2)
        N_d = 3 / 2 * (dx ** 2 + dy ** 2) ** (1 / 2) * (2 * dx * ddx + 2 * dy * ddy)
        return (Z_d * N - Z * N_d) / (N ** 2)

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx(s, 1)
        dy = self.sy(s, 1)
        yaw = math.atan2(dy, dx)
        return yaw

    def get_min_arc_length(self, p):
        """Get minimum arc length

        Calculate the closest arc length along the spline curve to position p

        :param p: position
        :return: arc length along the spline, minimum distance to p
        """
        x = p[0]
        y = p[1]

        def distance(s):
            """
            Calculate the distance between two points
            """
            sx, sy = self.calc_position(s)
            return (sx - x) ** 2 + (sy - y) ** 2

        bounds = [(self.s[0], self.s[-1])]

        res = minimize(fun=distance,
                       x0=self.s[0],
                       bounds=bounds)

        s = res.x[0]
        d = distance(s)

        return s, d

# EOF
