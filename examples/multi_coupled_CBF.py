#!/usr/bin/env python
"""
Tutorial example for simulating a virtual environment

Environment includes two pairs of fixed wing aircraft with unicycle dynamics
whch are programmed to fly past each other while maintaining connectivity
to their squadron, and avoiding collisions with all other aircraft
"""

import numpy as np
from systemcontrol.basic_systems import SingleUnicycle
from systemcontrol.CBF_systems import CoupleCBF
from systemcontrol.animate import Animate, Actor
from matplotlib import patches, transforms
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


class FixedWing(CoupleCBF, SingleUnicycle, Actor):
    """ simulated robot """

    def __init__(self, x, ch, color):
        CoupleCBF.__init__(self, x, ch=[])
        SingleUnicycle.__init__(self, x)
        Actor.__init__(self)

        self.color = color  # squadron color
        self.v = .4  # velocity constraint for barrier
        self.w = 1  # maximum angular velocity
        self.Ds = .5  # safety distance buffer between COM and obstacle
        self.Dcon = 7  # maximum distance that it needs to maintain with neighbors

    def nominal(self):
        """ simple nominal controller """
        return np.array([1, np.sin(0 - self.x[2])])

    def self_cons(self):
        """ actuation constraints for just itself """
        n = np.shape(self.g())[1]
        Ca1 = np.identity(n, dtype='float32')
        Ca2 = -np.identity(n, dtype='float32')
        Ca = np.hstack((Ca1, Ca2))
        ba = np.array([self.v, -self.w, -2, -self.w])
        # constraints  are self.v <= v <= 2, -self.w <= w <= -self.w
        return (Ca, ba)

    def set_ch(self, sys_list):
        """ set up couple barrier functions """
        self.sys_list = sys_list
        listofch = []  # list of barrier function constraints
        listofa = [lambda x: .1*x**3 for sys in sys_list]  # alpha functions
        listofsys = [[sys] for sys in sys_list]  # system list

        listofcon = []  # barrier function for connectivity maintanence
        v = self.v
        w = self.w

        for sys in sys_list:
            listofch.append((lambda k:
                             lambda x: ((x[0] - v/w*np.sin(x[2])) - (x[3] - k.v/k.w*np.sin(x[5])))**2 +
                                       ((x[1] + v/w*np.cos(x[2])) - (x[4] + k.v/k.w*np.cos(x[5])))**2 -
                                       (v/w + k.v/k.w + self.Ds + k.Ds)**2)(sys))

            # maintain connectivity with vehicles the same squadron color
            if self.color == sys.color:
                a = lambda x: x**3
                ch = (lambda k:
                      lambda x: (self.Dcon - v/w - k.v/k.w)**2 -
                                ((x[0] - v/w*np.sin(x[2])) - (x[3] - k.v/k.w*np.sin(x[5])))**2 -
                                ((x[1] + v/w*np.cos(x[2])) - (x[4] + k.v/k.w*np.cos(x[5])))**2)(sys)
                listofcon.append((ch, a, [sys]))

        colavoid = [e for e in zip(listofch, listofa, listofsys)]
        self.ch = colavoid + listofcon

    def uni_draw(self):
        """ function to draw a triangle representing the unicycle model"""

        w = .3  # width
        le = .75  # length
        th = self.x[2]  # direction

        # points for triangle for plotting
        offset = np.array([le/2*np.cos(th), le/2*np.sin(th)]).T
        p1 = self.x[0:2] + np.array([-w*np.sin(th), w*np.cos(th)]).T-offset
        p2 = self.x[0:2] + np.array([w*np.sin(th), -w*np.cos(th)]).T-offset
        p3 = self.x[0:2] + np.array([le/2*np.cos(th), le/2*np.sin(th)]).T

        return [p1, p2, p3]

    def draw_setup(self, axes):
        """ draw self """
        self.drawings.append(plt.Polygon(xy=self.uni_draw(), color=self.color))

    def draw_update(self, axes):
        """ update position """
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
    Red1 = FixedWing(np.array([-8, -.5, 0]), [], 'red')
    Red2 = FixedWing(np.array([-8, 2, 0.]), [],  'red')
    Blue1 = FixedWing(np.array([8, -2.5, np.pi]), [], 'blue')
    Blue2 = FixedWing(np.array([8, -.5, np.pi]), [], 'blue')

    # set coupled barriers beween fixed wings
    Red1.set_ch([Red2, Blue1, Blue2])
    Red2.set_ch([Red1, Blue1, Blue2])
    Blue1.set_ch([Red1, Red2, Blue2])
    Blue2.set_ch([Red1, Red2, Blue1])

    # initialize connections
    Con1 = Connection(Red1, Red2)
    Con2 = Connection(Blue1, Blue2)

    # set up list of systems that will be plotted
    sys_list = [Con1, Con2, Red1, Red2, Blue1, Blue2]

    # run animation
    animator = Animate(sys_list)
    animator.animate()
