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
        self.obstacles = obstacles  # list of obstacles
        self.r = r  # radius of obstacle
        self.v = .4  # velocity constraint for barrier
        self.w = 1  # maximum angular velocity
        self.Ds = .5  # safety distance buffer between COM and obstacle
        self.p = 3  # radius of limit cycle to converge to

        FeasibleCBF.__init__(self, x, self.seth(), self.seta())
        SingleUnicycle.__init__(self, x)
        Actor.__init__(self)

    def nominal(self):
        """
        controller to go in a circle
        set desired theta to converge to limit cycle
        """
        norm = np.minimum((self.x[0]**2 + self.x[1]**2)**.5, 2*self.p)
        td = np.arctan2(self.x[1], self.x[0]) + norm/self.p*np.pi/2

        u = np.array([self.v - .2, np.sin(td - self.x[2])])
        return u

    def input_cons(self):
        """ actuation constraints """
        n = np.shape(self.g())[1]
        Ca1 = np.identity(n, dtype='float32')
        Ca2 = -np.identity(n, dtype='float32')
        Ca = np.hstack((Ca1, Ca2))
        ba = np.array([self.v, -self.w, -2, -self.w])
        # constraints  are self.v <= v <= 2, -self.w <= w <= -self.w
        return (Ca, ba)

    def seth(self):
        """ set up barrier functions """
        listofh = []  # list of barrier function constraints
        v = self.v
        w = self.w

        for obs in self.obstacles:
            # create barrier functions
            # applied a nested lambda trick to get scopes to work out
            listofh.append((lambda y:
                            lambda x: (x[0] - y[0] - v/w*np.sin(x[2]))**2 +
                                      (x[1] - y[1] + v/w*np.cos(x[2]))**2 -
                                      (v/w + self.r + self.Ds)**2)(obs))
        return listofh

    def seta(self):
        """ alpha functions f(x) = x^3"""
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
        """ draw self """
        self.drawings.append(plt.Polygon(xy=self.uni_draw(), color='red'))

    def draw_update(self, axes):
        """ update position """
        self.drawings[0].set_xy(self.uni_draw())

if __name__ == '__main__':
    obstacles = [np.array([3, 2]),
                 np.array([-3, -2]),
                 np.array([-.5, 2.5])]  # obstacles
    r = .5  # radius of obstacles

    Car = SmartCar(np.array([6, -1, np.pi/2]), obstacles, r)  # initialize SmartCar
    sys_list = [Car]  # set up list of systems that will be plotted
    ob_list = [patches.Circle(obs, r) for obs in obstacles]  # plot obstacles

    animator = Animate(sys_list, ob_list)

    animator.animate()  # run  animation
