#!/usr/bin/env python
"""
Tutorial example for plotting multiple figures

Environment includes a double integrator car in one dimension
with acceleration and position being plotted.
"""

import numpy as np
from systemcontrol.basic_systems import ControlSystem, DrawSystem
from systemcontrol.animate import MultiAnimate
from matplotlib import patches, transforms
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


class FACTSCar(ControlSystem, DrawSystem):
    """ The FACTS Car! """

    def __init__(self, x):
        DrawSystem.__init__(self, x)  # used for drawing in the animation
        self.record = None  # record distance value

    def f(self):
        """x_dot = v """
        return np.array([self.x[1], 0])

    def g(self):
        """ v_dot = a """
        return np.array([0, 1.])

    def u(self):
        """ apply brakes if near wall"""
        if self.x[0] < -5:
            u = 0
        else:
            u = .1*(-.25 - self.x[0]) - self.x[1]  # PD controller

        return u

    def draw_setup(self, axes):
        """
        draw car
        and plot lines
        """
        self.drawings = [[], []]
        img = plt.imread('../files/Logo.png')  # read image to numpy array
        scale = .0025  # scale image down

        # set image size, this method is most stable from experience
        extent = [-img.shape[1]*scale, img.shape[1]*scale, -img.shape[0]*scale, img.shape[0]*scale]
        # set drawings to include image
        self.drawings[1] = [axes[1].imshow(img, extent=extent, animated=True)]
        # plot line for distance
        self.drawings[0] = [Line2D([], [], color='black')]
        # set title of first plot
        axes[0].set_title('Distance to Wall')

    def draw_update(self, axes):
        """
        update car and lines
        """
        x = self.x[0]  # x position

        if self.record is None:
            self.record = np.array([[self.t, -x]]).T
        else:  # append current distance to self.record array
            self.record = np.hstack((self.record, np.array([[self.t, -x]]).T))
        # set data for plotting line
        self.drawings[0][0].set_data(self.record)

        # set transform for image position
        tr = transforms.Affine2D().translate(x-3, 1)
        # don't forget to normalize against axes data
        self.drawings[1][0].set_transform(tr + axes[1].transData)


if __name__ == '__main__':

    start = np.array([-20., 1])  # start at -20 and 1 velocity

    FACTS = FACTSCar(start)  # initialize FACTS Car
    sys_list = [FACTS]  # set up list of systems that will be plotted
    # plot a road and a wall
    obs_list = [[], [patches.Rectangle((-23.5, -1.5), 25.5, 1.5, zorder=1, color='gray'),
                     patches.Rectangle((0, -1.5), 2, 7, color='black')]]
    # Used for Multiple Plots
    animator = MultiAnimate(sys_list,
                            obs_list,
                            plotarr=[2, 1],  # 2 X 1 plots
                            limits=[[0, 75, 0, 30], [-23, 2, -1, 4]])  # axis limits for each plot

    animator.animate()  # run  animation
