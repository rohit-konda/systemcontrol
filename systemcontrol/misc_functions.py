#!/usr/bin/env python
"""
Miscellarneous functions

from scipython.com
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from systemcontrol.basic_systems import *


def vecfield(system, lim=[-10, 10, -10, 10], res=1, ind=[0, 1]):
    """
    plot vector field for a system
    lim : [bottom, top, left, right]
    res : number of arrows per unit
    ind : index of state to be plotted
    """

    XX = np.linspace(lim[0], lim[1], math.ceil((lim[1] - lim[0])*res)+1)
    YY = np.linspace(lim[2], lim[3], math.ceil((lim[3] - lim[2])*res)+1)
    XY = [[x, y] for x in XX for y in YY]

    X = [elem[0] for elem in XY]
    Y = [elem[1] for elem in XY]
    U = [0]*len(X)
    V = [0]*len(X)

    for i in range(len(X)):
        system.x[ind[0]] = X[i]
        system.x[ind[1]] = Y[i]
        U[i] = system.f()[ind[0]]
        V[i] = system.f()[ind[1]]

    plt.quiver(X, Y, U, V)
    plt.show()
