#!/usr/bin/env python
"""
Tutorial example for simulating a virtual environment

Environment includes a double integrator robot that is initialized randomly
and uses a proportional controller to go to desired goal positions
"""

import numpy as np
from systemcontrol.basic_systems import SingleUnicycle
from systemcontrol.animate import Animate, DrawSystem
from matplotlib import patches, transforms


class FixedWing(DoubleIntegrator, DrawSystem):
    """ simulated robot """

    def __init__(self, x, goal):
        SingleUnicycle.__init__(self, x)  # extends the Double Integrator Class in basic_systems
        DrawSystem.__init__(self)  # used for drawing in the animation

        self.goal = goal  # desired x, y position

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


class Connection(DrawSystem):
    """ draws network between connection of aircrafts """
    def __init__(self, head, tail):
        self.head = head  # head of edge to first aircraft
        self.tail = tail  # tail of edge to second aircraft

    def draw_setup(self, axes):
        self.drawings.append()

    def draw_update(self, axes):
        self.drawings[0].set_xy(self.uni_draw())





if __name__ == '__main__':

    random_pos = 5*np.random.rand(4)  # random position and random velocity
    goal = [0, 0]  # go to the origin

    Red1 = FixedWing(random_pos, goal)  # initialize Robot
    Red2 = FixedWing()
    Blue1 = FixedWing()
    Blue2 = FixedWing()

    sys_list = [happy, Robot(5*np.random.rand(4), goal)]  # set up list of systems that will be plotted

    #  Note most of these parameters are set by default
    animator = Animate(sys_list, 

    animator.animate()  # run  animation
