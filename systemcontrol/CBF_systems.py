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

    def __init__(self, x, ch=[], h=[], a=[], G_self=None):
        """
        ch is list of tuples :(ch_i, ach_i, [sys_i]) for coupled barrier functions
        ch_i : coupled barrier function, takes multiple systems state as input
        ach_i: coupled alpha function
        [sys_i]: list of systems needed to evaluate ch_i
        """
        FeasibleCBF.__init__(self, x, h, a)
        self.ch = ch
        # initialize system list
        NetworkSystem.__init__(self, x, list(set([sys for chi in ch for sys in chi])))
        # initialize objective matrix
        if not G_self:
            self.G_self = np.identity(np.shape(self.g())[1])
        self.setG()

    def block_identity(self, arr_list):
        """ generate a diagonal block matric from an array list """
        row = sum([np.shape(arr)[0] for arr in arr_list])
        col = sum([np.shape(arr)[1] for arr in arr_list])
        blockarr = np.zeros((row, col))

        r = 0
        c = 0
        for arr in arr_list:
            ri, ci = np.shape(arr)
            blockarr[r:r+ri, c:c+ci] = arr
            r += ri
            c += ci
        return blockarr

    def setG(self):
        """
        set the objective function for decentralized calculation
        from list of system objective functions
        """
        G_list = [self.G_self] + [sys.G_self for sys in self.sys_list]
        self.G = self.block_identity(G_list)

    def input_cons(self):
        """
        set input constraints from combining  constraints from self_cons for 
        all the systems in the neighborhood
        """
        Ca_list = [self.self_cons()[0]] + [sys.self_cons()[0] for sys in self.sys_list]
        Ca = self.block_identity(Ca_list)
        ba = np.concatenate([self.self_cons()[1]] + [sys.self_cons()[1] for sys in self.sys_list])
        return Ca, ba

    def self_cons(self):
        """ constraint for self """
        return CBFSystem.input_cons(self)

    def u(self):
        """ feedback controller using coupled CBF """
        nom = self.nominal()

        if self.sys_list:
            sysnom = np.concatenate([sys.nominal() for sys in self.sys_list])
            ud = np.concatenate((nom, sysnom))
        else:
            ud = nom
        self.setG()
        u_opt = self.qp_u(ud)
        return u_opt[0:np.shape(self.g())[1]]

    def chCBF(self):
        """ Control barrier function for coupled system """
        m = len(self.ch)
        l = np.shape(self.g())[1]
        num = sum([np.shape(sys.g())[1] for sys in self.sys_list])

        C = np.zeros((num + l, m))
        b = np.zeros((m,))

        for i in range(m):
            chi = self.ch[i]
            x_hat_i = np.array([xh for sys in chi[2] for xh in sys.x])
            x_hat = np.concatenate((self.x, x_hat_i))

            gradient = self.calc_grad(chi[0], x_hat)
            h_dot = gradient[0:len(self.x)]
            h_dot_i = gradient[len(self.x):]

            Lfh = np.dot(h_dot, self.f())
            Lgh = np.dot(h_dot, self.g())

            c = 0
            Lfhi = 0.
            Lghi = np.zeros((num,))

            for sys in chi[2]:
                k = len(sys.x)
                Lfhi += np.dot(h_dot[c:c+k], sys.f())
                sys_st = [i_ for i_ in range(len(self.sys_list)) if self.sys_list[i_] == sys][0]
                sys_end = sys_st + np.shape(sys.g())[1]
                Lghi[sys_st:sys_end] = np.dot(h_dot_i[c:c+k], sys.g())
                c += k

            alpha = chi[1](chi[0](x_hat))
            C[0:l, i] = Lgh
            C[l:, i] = Lghi
            b[i] = -(alpha + Lfh + Lfhi)

        return C, b + self.epsilon

    def CBF(self):
        """ Control barrier function for concatenating couple and self CBF"""
        l = np.shape(self.g())[1]
        num = sum([np.shape(sys.g())[1] for sys in self.sys_list])

        Cf, bh = FeasibleCBF.CBF(self)
        Ch = np.vstack((Cf, np.zeros((num, np.shape(Cf)[1]))))

        if self.ch:
            Cch, bch = self.chCBF()
            C = np.hstack((Ch, Cch))
            b = np.concatenate((bh, bch))
        else:
            C = Ch
            b = bh
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
