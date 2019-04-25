#!/usr/bin/env python
"""
Tutorial example for simulating a virtual environment

Environment includes two pairs of fixed wing aircraft with unicycle dynamics
whch are programmed to fly past each other while maintaining connectivity
to their squadron, and avoiding collisions with all other aircraft
"""

import numpy as np
from systemcontrol.basic_systems import SingleUnicycle
from systemcontrol.animate import Animate, Actor
from matplotlib import patches, transforms
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


class FixedWing(SingleUnicycle, Actor):
    """ simulated robot """

    def __init__(self, x, color):
        SingleUnicycle.__init__(self, x)  # unicycle model
        Actor.__init__(self)  # used for drawing in the animation

        self.color = color

    def u(self):
        return np.array([1, 0])

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
        self.drawings.append(plt.Polygon(xy=self.uni_draw(), color=self.color))

    def draw_update(self, axes):
        self.drawings[0].set_xy(self.uni_draw())


class Connection(Actor):
    """
    draws network between connection of aircrafts
    Also an example of a drawn object that
    depends on multiple other objects
    """

    def __init__(self, head, tail):
        Actor.__init__(self)
        self.head = head  # head of edge to first aircraft
        self.tail = tail  # tail of edge to second aircraft

    def draw_setup(self, axes):
        # draw line
        self.drawings.append(Line2D([], [], color='black'))

    def draw_update(self, axes):
        # update line
        line = np.vstack((self.head.x[0:2], self.tail.x[0:2])).T
        self.drawings[0].set_data(line)


if __name__ == '__main__':

    # intialize aircraft
    Red1 = FixedWing(np.array([-8, 2., 0]), 'red')
    Red2 = FixedWing(np.array([-8, -2., 0]), 'red')
    Blue1 = FixedWing(np.array([8, 2.5, np.pi]), 'blue')
    Blue2 = FixedWing(np.array([8, -1.5, np.pi]), 'blue')

    # initialize connections
    Con1 = Connection(Red1, Red2)
    Con2 = Connection(Blue1, Blue2)

    # set up list of systems that will be plotted
    sys_list = [Con1, Con2, Red1, Red2, Blue1, Blue2]

    animator = Animate(sys_list)

    animator.animate()  # run  animation
