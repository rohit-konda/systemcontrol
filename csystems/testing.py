import numpy as np
from basic_systems import *
from CBF_systems import *
from animate import *


class Tester(FeasibleInfimum, DoubleUnicycle):
    """docstring for Tester"""
    def __init__(self, x, p, gamma, a, dirc, Tlim):
        FeasibleInfimum.__init__(self, x, p, gamma, a, Tlim)
        self.dirc = dirc
        self.simdt = .001

    def nominal(self):
        u1 = 10*(1 - self.x[2])
        u1 = 1
        u2 = 1*(np.sin((self.dirc - self.x[3])))
        return np.array([u1, u2])

    def input_cons(self):
        Ca = np.vstack((np.identity(2, dtype='float32'), -np.identity(2, dtype='float32'))).T
        ba = -np.ones((4,), dtype='float32')
        return (Ca, ba)

    # dynamics of control system
    def flow(self):
        self.u()
        u = np.array([0, 0])
        return self.f() + self.g() @ u

    def draw_setup(self, axes):
        self.drawings = [plt.Polygon(xy=self.uni_draw(), color='red'), plt.Polygon(xy=self.circle_draw(), fill=False)]

    def draw_update(self, axes):
        self.drawings[0].set_xy(self.uni_draw())
        self.drawings[1].set_xy(self.circle_draw())

    def uni_draw(self):
        w = .15
        l = .3
        th = self.x[3]
        offset = np.array([l/2*np.cos(th), l/2*np.sin(th)]).T
        p1 = self.x[0:2] + np.array([-w*np.sin(th), w*np.cos(th)]).T-offset
        p2 = self.x[0:2] + np.array([w*np.sin(th), -w*np.cos(th)]).T-offset
        p3 = self.x[0:2] + np.array([l*np.cos(th), l*np.sin(th)]).T
        return [p1, p2, p3]

    def circle_draw(self):
        r = self.x[2]
        x = self.x[0] + self.x[2]*np.sin(self.x[3])
        y = self.x[1] - self.x[2]*np.cos(self.x[3])
        theta = np.linspace(0, 2*np.pi, 30)
        return [np.array([x + r*np.cos(t), y + r*np.sin(t)]) for t in theta]


def a(h): return h


def p(x): return x[0]**2 + x[1]**2 - 1


def gamma(x): return np.array([0, -1])

if __name__ == '__main__':
    # set up models
    dt = 1/20

    u1 = Tester(np.array([-3.0, 0, 1, 0]), p, gamma, a, 0, 5)
    sys_list = [u1]

    anim = Animate(sys_list)
    anim.axes.add_artist(plt.Circle((0, 0), 1, fill=False))
    anim.animate()