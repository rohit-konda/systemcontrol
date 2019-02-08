import numpy as np
from CBF_systems import *
from animate import *
import matplotlib.patches as patches
import matplotlib.transforms as transforms


class Parking(FeasibleCBF, SingleUnicycle, DrawSystem):
    '''
    class for the parking problem
    '''
    def __init__(self, x, h, a):
        FeasibleCBF.__init__(self, x, h, None, a)
        SingleUnicycle.__init__(self, x)
        DrawSystem.__init__(self, x)

    def gradh(self):
        # hardcode gradient, return np.array() with 3 entries for each state
        pass

    def draw_setup(self, axes):
        self.drawings = [plt.Polygon(xy=self.uni_draw(), color='red')]

    def draw_update(self, axes):
        self.drawings[0].set_xy(self.uni_draw())

    def uni_draw(self):
        w = # width
        l = # length
        th = self.x[2]
        offset = np.array([l/2*np.cos(th), l/2*np.sin(th)]).T
        p1 = self.x[0:2] + np.array([-w*np.sin(th), w*np.cos(th)]).T-offset
        p2 = self.x[0:2] + np.array([w*np.sin(th), -w*np.cos(th)]).T-offset
        p3 = self.x[0:2] + np.array([l*np.cos(th), l*np.sin(th)]).T
        return [p1, p2, p3]

    def u(self):
        return np.array([1, 1])


# CBF
def h(x):
    return x[0] + x[1] + x[2]  # x + y + theta


# class K
def a(h):
    return h

if __name__ == '__main__':
    x = np.array([0., 0, 0])
    car = Parking(x, h, a)
    animator = Animate([car])
    animator.animate()
