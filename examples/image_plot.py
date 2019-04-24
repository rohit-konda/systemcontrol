#!/usr/bin/env python
"""
Tutorial example for animating images

Environment includes a double integrator helicopter that is initialized randomly
and uses a proportional controller to go to desired goal positions
"""

import numpy as np
from systemcontrol.basic_systems import DoubleIntegrator, DrawSystem
from systemcontrol.animate import Animate
from matplotlib import patches, transforms
import matplotlib.pyplot as plt


class Helicopter(DoubleIntegrator, DrawSystem):
    """ simulated helicopter """

    def __init__(self, x, goal):
        DoubleIntegrator.__init__(self, x)  # extends the Double Integrator Class in basic_systems
        DrawSystem.__init__(self, x)  # used for drawing in the animation

        self.goal = goal  # desired x, y position

    def u(self):
        """ proportional controller """

        xerr = (goal[0] - self.x[0])  # error for x position
        vxerr = -self.x[2]  # error for x velocity
        yerr = (goal[1] - self.x[1])  # error for y position
        vyerr = -self.x[3]  # error for y velocity

        K = .1  # gain for proportional controller for position
        u = np.array([K*xerr + vxerr, K*yerr + vyerr])  # controller
        return u  # input

    def draw_setup(self, axes):
        """ set up helicopter drawing """

        img = plt.imread('../files/helicopter.png')  # read image to numpy array
        scale = .0005  # scale image down

        # set image size, this method is most stable from experience
        extent = [-img.shape[1]*scale, img.shape[1]*scale, -img.shape[0]*scale, img.shape[0]*scale]
        # set drawings to include image
        self.drawings = [axes.imshow(img, extent=extent, animated=True)]

    def draw_update(self, axes):
        """
        change characteristics of drawing objects
        these objects will be in self.drawings
        """
        x = self.x[0]  # x position
        y = self.x[1]  # y position

        # set transform for image position
        tr = transforms.Affine2D().translate(x, y)
        # don't forget to normalize against axes data
        self.drawings[0].set_transform(tr + axes.transData)

if __name__ == '__main__':
    random_pos = 8*np.random.rand(4)  # randomize initial state
    random_pos[2:] = 0  # set initial velocity to 0
    goal = [0, 0]  # go to the origin

    heli = Helicopter(random_pos, goal)  # initialize Robot
    sys_list = [heli]  # set up list of systems that will be plotted
    ob_list = [patches.Circle([0, 0], 1.2, color='salmon', zorder=1)]  # platform

    animator = Animate(sys_list, ob_list)  # set up animator object

    animator.animate()  # run  animation
