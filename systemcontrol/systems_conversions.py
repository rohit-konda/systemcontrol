#!/usr/bin/env python
"""
Conversions between basic control systems

"""
import numpy as np
from systemcontrol.basic_systems import *


class Uni_Diffeo(SingleUnicycle):
    """ apply near identity diffeomorphism to control
    unicycle using cartesian control"""

    def __init__(self, x, l):
        self.l = l  # distance of point ahead to control with diffeomorphism
        SingleUnicycle.__init__(self, x)

    def u_diff(self):
        """ 2 inputs: desired v_x and v_y for the point ahead"""
        return np.array([0, 0])

    def u(self):
        """ inputs for unicyle : v and omega from using diffeomorphism """
        diffeo = np.array([[np.cos(self.x[2]), -self.l*np.sin(self.x[2])],
                           [np.sin(self.x[2]), self.l*np.cos(self.x[2])]])
        return np.dot(np.linalg.inv(diffeo), self.u_diff())

    def get_r(self):
        """ get position of the lookahead point """

        rx = self.x[0] + self.l*np.cos(self.x[2])
        ry = self.x[1] + self.l*np.sin(self.x[2])
        return np.array([rx, ry])
