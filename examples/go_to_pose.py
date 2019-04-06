#!/usr/bin/env python
"""
Tutorial example for simulating a virtual environment

Environment includes multiple double integrators that are initialized randomly in a 5x5 squares
and use a proportional controller to go to desired goal positions
"""

from systemcontrol.basic_systems import DoubleIntegrator
from systemcontrol.animate import Animate


class Robot(DoubleUnicycle, DrawSystem):
    """ simulated robot """

    def __init__(self, x, goal):
        DoubleUnicycle.__init__(self, x)
        DrawSystem.__init__(self, x)
        self.goal = goal  # desired x, y position

    def u(self):
        """ proportional controller """
        return np.array([-(x1 - goal[0]) - 1, -(x2 - goal[1]) - 1])

    def draw_setup(self, axes):
        """
        set up self.drawings : 
        for 
        """
        body = patches.Circle((self.x[0], self.x[1]), 1)
        eye1 = patches.Circle((self.x[0], self.x[1]), 1)
        eye2 = patches.Circle((self.x[0], self.x[1]), 1)
        smile = 

        self.drawings[0] = [body, eye1, eye2, smile]


    def draw_update(self, axes):
        """

        """
        self.drawings[0][0].center = (self.x[0], self.x[1])
        self.drawings[1][0].center = (self.x[0], self.x[1])
        tr = transforms.Affine2D().translate(self.x[0], self.x[1])
        self.drawings[0][1].set_transform(tr + axes[0].transData)
        #self.drawings[1][1].set_transform(tr + axes[0].transData)

if __name__ == '__main__':
    n = 5
    sys_list = []
    for i in range(n):
        sys_list.append(Robot())

    animator = Animate(sys_list, limits=[[-10, 10, -10, 10], [-10, 10, -10, 10]])
    animator.animate()

