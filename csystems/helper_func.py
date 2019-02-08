'''
Helper functions for plotting/analyzing control systems

from scipython.com
'''
import numpy as np
import matplotlib.pyplot as plt


# vector field
def vec_field(F):
    x = np.linspace(-10, 10, 30)
    y = np.linspace(-10, 10, 30)
    xv, yv = np.meshgrid(x, y)
    xx, yy = np.meshgrid(x, y)
    for i in range(len(x)):
        for j in range(len(y)):
            val = F(x[i], y[j])
            xx[i][j] = val[1]
            yy[i][j] = val[0]
    plt.quiver(xv, yv, xx, yy, zorder=1)
    ax = plt.gca()
    ax.add_artist(plt.Circle((0, 0), 6, zorder=2, fill=False))
    ax.add_artist(plt.Circle((0, 0), 4, zorder=2, fill=False))
    plt.show()


# Bivariable Function
def F(x, y):
    pd = 5
    pos = np.array([x, y]).T
    in_ = np.array([0, -1]).T
    out_ = np.array([0, 1]).T
    k = .1
    vec = -k/ang(in_, pos)*pos + k/ang(out_, pos)*pos + np.dot(R(-(90 + (90/5)*(p(x, y) - pd))), pos)
    t = np.arctan2(*np.flip(vec, 0))
    return np.array([np.cos(t), np.sin(t)]).T


# magnitude
def p(x, y): return (x**2 + y**2)**.5


# set u_list for each unicycle object based on adjacency matrix
def set_u_list(A, list_of_u):
    r, c = np.shape(A)
    for i in range(r):
        for j in range(c):
            if A[i][j] == 1:
                list_of_u[i].u_list.append(list_of_u[j])


# rotation matrix
def R(t):
    c, s = np.cos(t), np.sin(t)
    return np.array(((c, -s), (s, c)))


# norm
def n2(x): return np.dot(x, x)**.5


# angle between 2 vectors
def ang(x, y):
    if n2(x) == 0 or n2(y) == 0:
        return 0
    else:
        return np.arccos(np.dot(x, y)/n2(x)/n2(y))


# arccos with correct sign
def arccos2(px, py, p2):
    if py >= 0:
        return np.arccos(px/p2)
    else:
        return 2*np.pi - np.arccos(px/p2)


# draw vector field
def vec_field_system(sys_init):
        for i in range(-10, 10):
            for j in range(-10, 10):
                sys = sys_init(i, j)
                traj = sys.run(2)
                plt.plot(traj[0].T, traj[1].T)
                plt.plot(traj[0][0], traj[1][0], '.')
        ax = plt.gca()
        plt.show()
