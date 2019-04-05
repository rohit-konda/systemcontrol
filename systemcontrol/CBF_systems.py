#!/usr/bin/env python
"""
system formulations with control barrier architechtures
"""
import numpy as np
from math import ceil, log
from systemcontrol.basic_systems import *
from quadprog import solve_qp
import traceback


class CBFSystem(ControlSystem):
    """ System with Control Barrier Formulation """

    def __init__(self, x, G=None):
        ControlSystem.__init__(self, x)

        if not G:
            self.G = np.identity(np.shape(self.g())[1])
        self.default = lambda x: np.array([0., 0])

    def u(self):
        """ feedback controller using CBF """
        ud = self.nominal()  # u nominal
        u_opt = self.qp_u(ud)  # safe controller
        return u_opt

    def nominal(self):
        """ nominal controller """
        raise NotImplementedError

    def input_cons(self):
        """ actuation constraints """
        l = np.shape(self.g())[1]
        Ca = np.zeros((l, 1), dtype='float32')
        ba = np.array([-1.])
        return (Ca, ba)

    def CBF(self):
        """ control barrier: returns safety constraints """
        raise NotImplementedError

    def qp_u(self, ud):
        """ QP solver """
        Cc, bc = self.CBF()
        Ca, ba = self.input_cons()
        # parameters for solve_qp

        A = np.hstack((Cc, Ca))
        b = np.concatenate((bc, ba))
        G = self.G
        aT = 1/2 * ud @ (self.G + self.G.T)
        try:
            u_opt = solve_qp(self.G, aT, A, b)[0]
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

    def gradh(self, h):
        """ numerical calculation of gradient of h """
        x_cop = np.copy(self.x)
        n = len(self.x)
        grad = np.zeros((n,))
        step = .0001

        for i in range(n):
            dh = []
            for dx in [-step, step]:
                dx_state = self.x[i] + dx
                x_cop[i] = dx_state
                dh.append(h(x_cop))
            # median approximation
            grad[i] = np.diff(dh)/(2*step)
        return grad

    def CBF(self):
        """ control barrier function """
        C = np.zeros((np.shape(self.g())[1], len(self.h)))
        b = np.zeros((len(self.h),))
        for i in range(len(self.h)):
            h_dot = self.gradh(self.h[i])
            Lfh = h_dot @ self.f()
            Lgh = h_dot @ self.g()
            alpha = self.a[i](self.h[i](self.x))
            C[:, i] = np.array(Lgh)
            b[i] = -(alpha + Lfh)
        return C, b + self.epsilon


class CoupleCBF(FeasibleCBF, NetworkSystem):
    """ Feasible Control Barrier Formulations for Coupled Systems """

    def __init__(self, x, h=[], ch=None, sys_list=[], a=[], ach=None):
        FeasibleCBF.__init__(self, x, h, a)
        NetworkSystem.__init__(self, x, sys_list)
        self.ch = ch  # barrier between the coupled system
        self.ach = ach  # class K function for coupled barrier

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

    def gradch(self, ch,  j):
        """ numerical calculation of gradient of barrier between 2 systems """
        xi = np.copy(self.x)
        xj = np.copy(self.sys_list[j].x)
        n = len(xi)
        grad = np.zeros((2*n,))
        step = .001

        for i in range(2*n):
            dh = []
            for dx in [-step, step]:
                if i < n:
                    dxstate = self.x[i] + dx
                    xi[i] = dxstate
                else:
                    dxstate = self.sys_list[j].x[i-n] + dx
                    xj[i-n] = dxstate
                dh.append(ch(xi, xj))
            # median approximation
            grad[i] = np.diff(dh)/(2*step)
        return grad

    def chCBF(self):
        """ Control barrier function for coupled system """
        C = np.zeros((len(self.g()[1])*(length + 1), length))
        b = np.zeros((length,))

        for j in range(length):
            sysj = self.sys_list[j]

            gradient = self.gradch(j)
            h_dot = gradient[0:len(self.x)]
            h_dot_j = gradient[len(self.x):]

            Lfh = h_dot @ self.f()
            Lgh = h_dot @ self.g()

            Lfhj = h_dot_j @ sysj.f()
            Lghj = h_dot_j @ sysj.g()

            alpha = self.ach((self.ch(self.x, sysj.x)))

            l = len(self.g()[1])

            C[0:l, j] = Lgh
            C[l*(j+1):l*(j+2), j] = Lghj

            b[j] = -(alpha + Lfh + Lfhj)

        return C, b + self.epsilon

    def CBF(self):
        """ Control barrier function for single system """
        l = np.shape(self.g())[1]
        n = l*(len(self.sys_list)+1)
        if self.h:
            Ch, bh = FeasibleCBF.CBF()
            temp = np.zeros((n,))
            temp[0:l] = Ch
            Ch = np.reshape(temp, (n, 1))
        else:
            Ch, bh = (np.zeros((n, 1)), np.array([0]))
        if self.ch and self.sys_list:
            Cch, bch = self.chCBF()
        else:
            Cch, bch = (np.zeros((n, 1)), np.array([0]))

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
