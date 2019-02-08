import numpy as np
from basic_systems import *
from animate import *
import matplotlib.patches as patches
import matplotlib.transforms as transforms


class Tester(DoubleUnicycle, DrawSystem):
    def __init__(self, x):
        DoubleUnicycle.__init__(self, x)
        DrawSystem.__init__(self, x)

    def u(self):
        return np.array([0, 0])

    def draw_setup(self, axes):
        self.drawings = [[patches.Circle((self.x[0], self.x[1]), 1)], []]
        img = plt.imread('C:\\Users\\rohit\\Documents\\Research\\Programs\\Sim_work\\Sim_Files\\vector images\\Cars\\car_3.png')
        scale = .0009
        extent = [-img.shape[1]*scale, img.shape[1]*scale, -img.shape[0]*scale, img.shape[0]*scale]
        self.drawings[0] = [patches.Circle((self.x[0], self.x[1]), 1), axes[0].imshow(img, extent=extent, animated=True)]
        self.drawings[1] = [patches.Circle((self.x[0], self.x[1]), 1)]

    def draw_update(self, axes):
        self.drawings[0][0].center = (self.x[0], self.x[1])
        self.drawings[1][0].center = (self.x[0], self.x[1])
        tr = transforms.Affine2D().translate(self.x[0], self.x[1])
        self.drawings[0][1].set_transform(tr + axes[0].transData)
        #self.drawings[1][1].set_transform(tr + axes[0].transData)


if __name__ == '__main__':
    sys = Tester(np.array([0, 0, .1, 0]))
    sys_list = [sys]

    animator = MultiAnimate(sys_list, plotarr=[1, 2], limits=[[-10, 10, -10, 10], [-10, 10, -10, 10]])

    animator.animate()
