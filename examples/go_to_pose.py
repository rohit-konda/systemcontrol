#!/usr/bin/env python
"""
Tutorial example for simulating a virtual environment

Environment includes a double integrator robot that is initialized randomly
and uses a proportional controller to go to desired goal positions
"""

import numpy as np
from systemcontrol.basic_systems import DoubleIntegrator
from systemcontrol.animate import Animate, Actor
from matplotlib import patches


class Robot(DoubleIntegrator, Actor):
    """ simulated robot """

    def __init__(self, x, goal):
        DoubleIntegrator.__init__(self, x)  # extends the Double Integrator Class in basic_systems
        Actor.__init__(self)  # used for drawing in the animation

        self.goal = goal  # desired x, y position

    def u(self):
        """ proportional controller """

        xerr = (goal[0] - self.x[0])  # error for x position
        vxerr = -self.x[2]  # error for x velocity
        yerr = (goal[1] - self.x[1])  # error for y position
        vyerr = -self.x[3]  # error for y velocity

        K = .1  # gain for proportional controller for position

        u = np.array([K*xerr + vxerr, K*yerr + vyerr])  # controller

        return u

    def draw_setup(self, axes):
        """
        in this method from Actor, initialize all matplotlib objects for plotting
        to get info on figure axes, check axes parameter
        set self.drawings to a list that is plotted at each time point
        """
        x = self.x[0]  # x position
        y = self.x[1]  # y position

        # draw smiley
        body = patches.Circle((x, y), 1, fill=False, zorder=10)
        eye1 = patches.Circle((x, y), .1, fill=False, zorder=10)
        eye2 = patches.Circle((x, y), .1, fill=False, zorder=10)
        smile = patches.Arc((x, y), .8, .8, theta1=220, theta2=320)

        self.drawings = [body, eye1, eye2, smile]

    def draw_update(self, axes):
        """
        in this method from Actor, change characteristics of drawing objects
        these objects will be in self.drawings
        """
        x = self.x[0]  # x position
        y = self.x[1]  # y position
        offy = .3
        offx = .3

        # move smiley to right position
        self.drawings[0].center = (x, y)
        self.drawings[1].center = (x + offx, y + offy)
        self.drawings[2].center = (x - offx, y + offy)
        self.drawings[3].center = (x, y)


if __name__ == '__main__':

    random_pos = 5*np.random.rand(4)  # random position and random velocity
    goal = [0, 0]  # go to the origin

    happy = Robot(random_pos, goal)  # initialize Robot
    sys_list = [happy]
    ob_list = [patches.Circle([0, 0], 1.1, fill=False, lw=2, color='navy')]

    #  Note most of these parameters are set by default
    animator = Animate(sys_list,  # set up list of systems that will be plotted
                       ob_list=ob_list,  # list of objects that are static
                       size=(10, 9),  # size of figure
                       xlim=[-10, 10],  # plotting limits of x axis
                       ylim=[-10, 10],  # plotting limits of x axis
                       showfig=True,  # set True to display animation
                       saveData=False,  # set True to save data as json
                       inter=False,  # set True to approximate real time animation
                       tight=True,  # axes padding decreases
                       dt=.1  # time step
                       )  # intialize simulation environment

    animator.animate()  # run animation
