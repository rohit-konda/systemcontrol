#!/usr/bin/env python
"""
Class defintions for animating control affine systems

from jakevdp.github.io animation tutorial and nickcharlton.net patch tutorial
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from time import time
from datetime import datetime
from systemcontrol.basic_systems import *
import json


class Animate():
    """ Parent Class for animating systems """

    def __init__(self, sys_list, ob_list=[], size=(18, 9), xlim=[-10, 10], ylim=[-10, 10],
                 showfig=True, saveData=False, inter=False):
        self.sys_list = sys_list  # list of system objects
        self.ob_list = ob_list  # list of objects that are static
        self.fig = plt.figure()  # figure handle
        self.size = size  # size of figure
        self.axes = None  # axes handle
        self.xlim = xlim  # x axis limits
        self.ylim = ylim  # y axis limits
        self.interval = 0  # interval to run in real time
        self.drawings = []  # drawings
        self.data = []  # state history
        self.time_text = None  # time bar
        self.grid_on = False  # turn on grid on plot
        self.saveData = saveData  # option of saving traces of animation
        self.showfig = showfig  # show Figure or not
        self.time = 0
        self.dt = self.sys_list[0].dt

        # setup axes
        self.setup()
        if inter:
            self.set_interval()

    def init_axes(self):
        """ initialization function, sets objects for axes """
        for sys in self.sys_list:
            sys.draw_setup(self.axes)
            self.drawings += sys.drawings

        self.drawings += self.ob_list

        for artist in self.drawings:
            self.axes.add_artist(artist)
        self.axes.images = []  # flush duplicate images

        self.time_text.set_text('')
        if self.saveData:
            current = np.hstack(tuple([sys.x for sys in self.sys_list]))
            self.data = current

        return (self.time_text,) + tuple(self.drawings)

    def update_frame(self, i):
        """ plot update function """
        for sys in self.sys_list:
            sys.step()
            sys.draw_update(self.axes)

        self.time_text.set_text('time = %.1f' % self.time)
        self.time += self.dt
        if self.saveData:
            current = np.hstack(tuple([sys.x for sys in self.sys_list]))
            self.data = np.vstack((self.data, current))

        return (self.time_text,) + tuple(self.drawings)

    def setup(self):
        """ setup figure handles """
        self.axes = self.fig.add_subplot(111, aspect='equal', autoscale_on=False,
                                         xlim=self.xlim, ylim=self.ylim)
        self.fig.set_size_inches(self.size)

        if self.grid_on:
            self.axes.grid()
        self.time_text = self.axes.text(0.02, 0.95, '', transform=self.axes.transAxes)

        self.init_axes()

    def set_interval(self):
        """ set up interval for approximately real time simulation """

        t0 = time()
        self.update_frame(0)
        t1 = time()
        a_time = 1000*self.dt - (t1 - t0)
        if a_time > 0:
            self.interval = a_time

    def animate(self, name=None, frames=500):
        """ animate function, if name specified, saves file """

        # run animation
        anim_handle = animation.FuncAnimation(self.fig, self.update_frame, frames=frames,
                                              interval=self.interval, blit=True)

        if name:
            file_name = str(name)+'.mp4'
            anim_handle.save(file_name, fps=30, extra_args=['-vcodec', 'h264',
                             '-pix_fmt', 'yuv420p'])
        elif self.showfig:
            plt.show()

        if self.saveData:
            fn = 'Simulation_at_' + ''.join(e for e in str(datetime.now())[:20] if e.isalnum())
            with open(fn, 'w') as fh:
                json.dump(self.data.tolist(), fh)


class MultiAnimate(Animate):
    """ For animating multiple figures """

    def __init__(self, sys_list, plotarr=[1, 1], limits=[[-10, 10, -10, 10]],
                 size=(18, 9), showfig=True, saveData=False, inter=False):

        self.plotarr = plotarr  # array of plots: M x N subplots
        self.limits = limits  # limits for each plot, [x_lower, x_upper, y_lower, y_upper]
        Animate.__init__(self, sys_list, size=(18, 9), showfig=True,
                         saveData=False, inter=False)

    def init_axes(self):
        """ initialization function, sets objects for axes """
        for sys in self.sys_list:
            sys.draw_setup(self.axes)
            for i in range(len(self.drawings)):
                self.drawings[i] += sys.drawings[i]

        for i in range(len(self.drawings)):
            for artist in self.drawings[i]:
                self.axes[i].add_artist(artist)
            self.axes[i].images = []  # flush duplicate images

        for time_text in self.time_text:
            time_text.set_text('')
        if self.saveData:
            current = np.hstack(tuple([sys.x for sys in self.sys_list]))
            self.data = current

        return tuple(self.time_text) + tuple([art for draw in self.drawings for art in draw])

    def update_frame(self, i):
        """ plot update function """
        for sys in self.sys_list:
            sys.step()
            sys.draw_update(self.axes)

        for time_text in self.time_text:
            time_text.set_text('time = %.1f' % self.time)
        self.time += self.dt
        if self.saveData:
            current = np.hstack(tuple([sys.x for sys in self.sys_list]))
            self.data = np.vstack((self.data, current))

        return tuple(self.time_text) + tuple([art for draw in self.drawings for art in draw])

    def setup(self):
        """ setup figure handles """
        # override Animate settings
        M = self.plotarr[0]
        N = self.plotarr[1]
        self.axes = []
        self.drawings = [[] for i in range(M*N)]
        self.time_text = []

        # add axes objects
        for i in range(M*N):
            xlim = self.limits[i][0:2]
            ylim = self.limits[i][2:4]
            self.axes.append(self.fig.add_subplot(M, N, i+1, aspect='equal',
                             autoscale_on=False, xlim=xlim, ylim=ylim))
            self.time_text.append(self.axes[i].text(0.02, 0.95, '', transform=self.axes[i].transAxes))

        self.fig.set_size_inches(self.size)
        self.init_axes()
