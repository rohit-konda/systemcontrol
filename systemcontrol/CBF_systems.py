#!/usr/bin/env python
"""
system formulations with control barrier architechtures
"""
import numpy as np
from math import ceil, log
from systemcontrol.basic_systems import *
from quadprog import solve_qp
import traceback
import matplotlib.pyplot as plt


class CBFSystem(ControlSystem):
    """ System with Control Barrier Formulation """

    def __init__(self, x, G=None):
        ControlSystem.__init__(self, x)

        if not G:
            self.G = np.identity(np.shape(self.g())[1])
        self.default = lambda x: np.zeros((np.shape(self.g())[1],))

    def u(self):
        """ feedback controller using CBF """
        ud = self.nominal()  # u nominal
        u_opt = self.qp_u(ud)  # safe controller
        return u_opt

    def nominal(self):
        """ nominal controller """
        raise NotImplementedError

    def input_cons(self):
        """
        actuation constraints
        Ca: n x m array for constraints
        ba: m, vector of constraints
        """
        l = np.shape(self.g())[1]
        Ca = np.zeros((l, 1), dtype='float32')
        ba = np.array([-1.])
        return (Ca, ba)

    def CBF(self):
        """ control barrier: returns safety constraints """
        raise NotImplementedError

    def qp_u(self, ud):
        """
        QP solver min x.T G x - a.T x subject to A.T x >= b
        in this case: (u - ud).T G (u - ud) is quadratic objective
        A =  n x m array of m constraints
        b = m, vector for constraints
        G = n x n objective quadratic term
        a = n, objective linear term 

        """
        Cc, bc = self.CBF()
        Ca, ba = self.input_cons()
        # parameters for solve_qp

        A = np.hstack((Cc, Ca))
        b = np.concatenate((bc, ba))
        G = self.G
        a = 1/2 * np.dot(ud, (self.G + self.G.T))
        try:
            u_opt = solve_qp(self.G, a, A, b)[0]
        except:
            traceback.print_exc()
            # default controller for error handling
            u_opt = self.default(self.x)
        return u_opt


class FeasibleCBF(CBFSystem):
    """ Feasible Control Barrier Formulation """

    def __init__(self, x, h=[], a=[]):
        CBFSystem.__init__(self, x)
        self.h = h  # list of barrier functions
        self.a = a  # list of class K functions
        self.epsilon = 0.0  # higher epsilon promotes conservatism

    def calc_grad(self, f, x_hat, step=.0001):
        """ numerical calculation of gradient of f at x_hat """
        n = len(x_hat)
        grad = np.zeros((n,))

        for i in range(n):
            df = []
            for dx in [-step, step]:
                temp = x_hat[i]
                x_hat[i] += dx
                df.append(f(x_hat))
                x_hat[i] = temp
            # median approximation
            grad[i] = np.diff(df)/(2*step)
        return grad

    def CBF(self):
        """ control barrier function """
        if self.h == []:
            C = np.zeros((np.shape(self.g())[1], 1))
            b = -np.ones((1,))
            return C, b

        C = np.zeros((np.shape(self.g())[1], len(self.h)))
        b = np.zeros((len(self.h),))
        for i in range(len(self.h)):
            h_dot = self.calc_grad(self.h[i], np.copy(self.x))
            Lfh = np.dot(h_dot, self.f())
            Lgh = np.dot(h_dot, self.g())
            alpha = self.a[i](self.h[i](self.x))
            C[:, i] = np.array(Lgh)
            b[i] = -(alpha + Lfh)
        return C, b + self.epsilon


class CoupleCBF(FeasibleCBF, NetworkSystem):
    """
    Decentralized Formulation for barrier functions
    for centralized, just use appended system and FeasibleCBF
    """

    def __init__(self, x, ch=[], h=[], a=[]):
        FeasibleCBF.__init__(self, x, h, a)

        # list of tuples : (ch_i, ach_i, [sys_i]) for coupled barrier functions
        # ch_i : coupled barrier function, takes multiple systems state as input
        # ach_i: coupled alpha function
        # [sys_i]: list of systems needed to evaluate ch_i
        self.ch = ch
        NetworkSystem.__init__(x, list(set([sys for chi in ch for sys in chi])))

    def u(self):
        """ feedback controller using coupled CBF """
        nom = self.nominal()

        if self.sys_list:
            sysnom = np.concatenate([sys.nominal() for sys in self.sys_list])
            ud = np.concatenate((nom, sysnom))
        else:
            ud = nom
        u_opt = self.qp_u(ud)
        return u_opt[0:np.shape(self.g())[1]]

    def chCBF(self):
        """ Control barrier function for coupled system """
        m = len(self.ch)
        C = np.zeros((len(self.g()[1])*(m + 1), m))
        b = np.zeros((m,))

        for j in range(m):

            gradient = self.calc_grad(self.ch[i][0])
            h_dot = gradient[0:len(self.x)]
            h_dot_j = gradient[len(self.x):]

            Lfh = np.dot(h_dot, self.f())
            Lgh = np.dot(h_dot, self.g())

            Lfhj = np.dot(h_dot_j, sysj.f())
            Lghj = np.dot(h_dot_j, sysj.g())


            alpha = self.ch[i][1](chi)

            l = len(self.g()[1])

            C[0:l, j] = Lgh
            C[l*(j+1):l*(j+2), j] = Lghj

            b[j] = -(alpha + Lfh + Lfhj)

        return C, b + self.epsilon

    def CBF(self):
        """ Control barrier function for single system """
        l = np.shape(self.g())[1]
        n = l*(len(self.sys_list)+1)

        Ch, bh = FeasibleCBF.CBF()
        temp = np.zeros((n,))
        temp[0:l] = Ch
        Ch = np.reshape(temp, (n, 1))

        if self.ch:
            Cch, bch = self.chCBF()
        else:
            Cch = np.zeros((n, 1))
            bch = np.array([0])

        C = np.hstack((Ch, Cch))
        b = np.concatenate((bh, bch))
        return C, b


class FeasibleInfimum(FeasibleCBF, SimulationSystem):
    """ Control Barrier Function that estimates infimum """

    def __init__(self, x, gamma, p=[], a=[], Tlim=[]):
        FeasibleCBF.__init__(self, x, [self.give_h(p, T) for elem, T in zip(p, Tlim)], a)
        SimulationSystem.__init__(self, x, gamma)

        self.p = p  # list of safety function
        self.Tlim = Tlim  # list of Time horizons of integration

    def give_h(self, p, Tlim):
        """ feasible calculation of barrier value from simulated system """
        def h(x):
            self.reset(x)
            infimum = None
            while self.simt < Tlim:
                val = p(self.simx)
                if not infimum or val < infimum:
                    infimum = val
                self.gamma_step()
            return infimum
        return h
