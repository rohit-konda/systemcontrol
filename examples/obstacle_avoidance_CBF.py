#!/usr/bin/env python
"""
Tutorial example for using barrier functions for collision avoidance

Environment includes a unicycle model of a car,
and some obstacles that the car has to avoid
"""

import numpy as np
from systemcontrol.basic_systems import SingleUnicycle
from systemcontrol.CBF_systems import FeasibleCBF
from systemcontrol.animate import Animate, Actor
import matplotlib.pyplot as plt
from matplotlib import patches


class SmartCar(FeasibleCBF, SingleUnicycle, Actor):
    """ car example that is programmed to avoid
    certain placed obstacled and go in a circle """

    def __init__(self, x, obstacles, r):
        self.obstacles = obstacles
        self.r = r
        self.v = .5

        FeasibleCBF.__init__(self, x, self.seth(), self.seta())
        SingleUnicycle.__init__(self, x)
        Actor.__init__(self)

    def u(self):
        """ controller to go in a circle"""
        K = 1
        p = 2.5
        norm = np.minimum((self.x[0]**2 + self.x[1]**2)**.5, 2*p)
        td = np.arctan2(self.x[1], self.x[0]) + norm/p*np.pi/2
        u = np.array([self.v, 1])
        return u

    def seth(self):
        listofh = []
        w = 1

        for obs in self.obstacles:
            listofh.append(lambda x:  (self.r - self.v/w)**2 -
                                      (self.x[0] - obs[0] - self.v/w*np.sin(self.x[2]))**2 -
                                      (self.x[1] - obs[1] + self.v/w*np.cos(self.x[2]))**2)
        return listofh

    def seta(self):
        return [lambda x: x**3 for obs in obstacles]

    def uni_draw(self):
        """ function to draw a triangle representing the unicycle model"""

        w = .3  # width
        l = .75  # length
        th = self.x[2]  # direction

        # points for triangle for plotting
        offset = np.array([l/2*np.cos(th), l/2*np.sin(th)]).T
        p1 = self.x[0:2] + np.array([-w*np.sin(th), w*np.cos(th)]).T-offset
        p2 = self.x[0:2] + np.array([w*np.sin(th), -w*np.cos(th)]).T-offset
        p3 = self.x[0:2] + np.array([l/2*np.cos(th), l/2*np.sin(th)]).T

        return [p1, p2, p3]

    def draw_setup(self, axes):
        self.drawings.append(plt.Polygon(xy=self.uni_draw(), color='red'))

    def draw_update(self, axes):
        self.drawings[0].set_xy(self.uni_draw())

if __name__ == '__main__':
    obstacles = [np.array([3, 2])]  # obstacles
    r = .5  # radius of obstacles

    Car = SmartCar(np.array([3, .7, np.pi/2]), obstacles, r)  # initialize SmartCar
    sys_list = [Car]  # set up list of systems that will be plotted
    ob_list = [patches.Circle(obs, r) for obs in obstacles]

    animator = Animate(sys_list, ob_list)

    animator.animate()  # run  animation
