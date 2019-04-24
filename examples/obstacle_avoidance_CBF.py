#!/usr/bin/env python
"""
Tutorial example for using barrier functions for collision avoidance

Environment includes a unicycle model of a car,
and some obstacles that the car has to avoid
"""

import numpy as np
from systemcontrol.basic_systems import SingleUnicycle, DrawSystem
from systemcontrol.CBF_systems import FeasibleCBF
from systemcontrol.animate import Animate
import matplotlib.pyplot as plt
from matplotlib import patches


class SmartCar(FeasibleCBF, SingleUnicycle, DrawSystem):
    """ car example that is programmed to avoid
    certain placed obstacled and go in a circle """

    def __init__(self, x, obstacles, r):
        self.obstacles = obstacles
        self.r = r
        FeasibleCBF.__init__(self, x, self.seth(), self.seta())
        SingleUnicycle.__init__(self, x)
        DrawSystem.__init__(self, x)

    def nominal(self):
        """ controller to go in a circle"""
        K = 1
        p = 3
        norm = np.maximum(self.x[0]**2 + self.x[1]**2, 2*p)
        td = np.arctan2(self.x[1], self.x[0]) + np.pi/2 + np.pi/2/p*(norm - p)

        u = np.array([.5, K*np.sin(td - self.x[2])])
        return u

    def seth(self):
        listofh = []
        #for obs in self.obstacles:
        #    listofh.append(lambda x: (obs[0] - self.x[0])**2 - (obs[1] - self.x[1])**2 - self.r**2)
        return listofh

    def seta(self):
        return [] # [lambda x: x for obs in obstacles]

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
    obstacles = [np.array([2.5, 2.5])]  # obstacles
    r = .5  # radius of obstacles

    Car = SmartCar(np.array([2, 0, np.pi/2]), obstacles, r)  # initialize SmartCar
    sys_list = [Car]  # set up list of systems that will be plotted
    ob_list = [patches.Circle(obs, r) for obs in obstacles]

    animator = Animate(sys_list, ob_list)

    animator.animate()  # run  animation
