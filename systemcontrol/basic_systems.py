#!/usr/bin/env python
"""
Class defintions for creating basic feedback control affine systems

"""
import numpy as np


class System:
    """general formulation for a continuous system"""

    def __init__(self, x, dt=.1, params=None, t=0):
        self.x = x  # state
        self.t = t  # time
        self.y = self.output() # output
        self.dt = dt  # time step
        self.params = params  # define parameters

    # x dot = f(t, x)
    def flow(self):
        """ dynamics of system """
        return self.f()

    def f(self):
        """ drift term """
        pass

    def output(self):
        """ set output of the system """
        pass

    def step(self):
        """ euler approximation integration"""
        self.x += self.flow()*self.dt
        self.y = self.output()
        self.t += self.dt

    def runge_kutta_step(self):
        """ RK 4th order approximation integration """
        state = np.copy(self.x)
        k1 = self.flow()*self.dt
        self.x = state + .5*k1
        k2 = self.flow()*self.dt
        self.x = state + .5*k2
        k3 = self.flow()*self.dt
        self.x = state + k3
        k4 = self.flow()*self.dt
        self.x = state + (k1+2*k2+2*k3+k4)/6
        self.y = self.output()
        self.t += self.dt

    def run(self, t):
        """ get trajectory from initial value for t time"""
        temp_x = self.x
        temp_t = self.t
        traj = np.zeros((len(self.x), int(t/self.dt)))
        for i in range(len(traj.T)):
            traj.T[i] = self.x
            self.step()
        self.x = temp_x
        self.t = temp_t
        self.y = self.output()
        return traj


class ControlSystem(System):
    """ general control affine system """

    def __init__(self, x, dt=.1, params=None, t=0):
        System.__init__(self, x, dt, params, t)

    # x dot = f(t, x) + g(t, x)*u(t, x)
    def flow(self):
        """ dynamics of control system """
        return self.f() + np.dot(self.g(), self.u())

    def g(self):
        """ control affine term"""
        pass

    def u(self):
        """ input """
        pass


class Linear(ControlSystem):
    """ time invariant linear system """

    def __init__(self, x, A, B, C, D):
        ControlSystem.__init__(self, x)
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def f(self):
        """ linear system """
        return np.dot(self.A, self.x)

    def g(self):
        """ control affine term"""
        return self.B

    def output(self):
        """ y = Cx + Du """
        return np.dot(self.C, self.x) + np.dot(self.D, self.u())


class SingleIntegrator(ControlSystem):
    """ single integrator control system with x, y """

    def __init__(self, x):
        ControlSystem.__init__(self, x)

    def f(self):
        """ single integrator dynamics """
        xdot = 0
        ydot = 0
        return np.array([xdot, ydot]).T

    def g(self):
        """ control affine term"""
        return np.identity(2)

    def u(self):
        """ 2 inputs: v_x , v_y """
        return np.array([0, 0])


class DoubleIntegrator(ControlSystem):
    """ double integrator control system with x, y, vx, and vy """

    def __init__(self, x):
        ControlSystem.__init__(self, x)

    def f(self):
        """ double integrator dynamics """
        xdot = self.x[2]
        ydot = self.x[3]
        vxdot = 0
        vydot = 0
        return np.array([xdot, ydot, vxdot, vydot]).T

    def g(self):
        """ control affine term"""
        return np.array([[0, 0, 1, 0], [0, 0, 0, 1]]).T

    def u(self):
        """ 2 inputs: acc_x , acc_y """
        return np.array([0, 0])


class SingleUnicycle(ControlSystem):
    """ unicycle control system with states x, y, th """

    def __init__(self, x):
        ControlSystem.__init__(self, x)

    def f(self):
        """ single unicycle dynamics """
        return np.array([0, 0, 0]).T

    def g(self):
        """ control affine term"""
        return np.array([[np.cos(self.x[2]), np.sin(self.x[2]), 0], [0, 0, 1]]).T

    def step(self):
        """ step function that wraps theta """
        ControlSystem.step(self)
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

    def u(self):
        """ 2 inputs: velocity, angular velocity """
        return np.array([0, 0])


class DoubleUnicycle(ControlSystem):
    """ double integrator unicycle control system with states  x, y, v, th """

    def __init__(self, x):
        ControlSystem.__init__(self, x)

    def f(self):
        """ double unicycle dynamics """
        xdot = self.x[2]*np.cos(self.x[3])
        ydot = self.x[2]*np.sin(self.x[3])
        vdot = 0
        thdot = 0
        return np.array([xdot, ydot, vdot, thdot]).T

    def g(self):
        """ control affine term"""
        return np.array([[0, 0, 1, 0], [0, 0, 0, 1]]).T

    def step(self):
        ControlSystem.step(self)
        self.x[3] = np.arctan2(np.sin(self.x[3]), np.cos(self.x[3]))

    def u(self):
        """ 2 inputs: acceleration, angular velocity """
        return np.array([0, 0])


class NetworkSystem(System):
    """ Parent class for systems with interactions """

    def __init__(self, x, sys_list):
        System.__init__(self, x)
        self.sys_list = sys_list


class SimulationSystem(ControlSystem):
    """ Parent class for generating traces for desired input """

    def __init__(self, x, gamma, simx=None):
        ControlSystem.__init__(self, x)
        self.gamma = gamma  # fake feedback controller
        self.simx = simx  # simulated state
        self.simt = 0  # simulated time
        self.simdt = self.dt  # simulated dt

    def gamma_flow(self, x):
        """ xdot for a given state """
        tempx = self.x.copy()
        self.x = x
        xdot = self.f() + np.dot(self.g(), self.gamma(x))
        self.x = tempx
        return xdot

    def gamma_step(self):
        """ flow for simx using gamma """
        self.simx += self.gamma_flow(self.simx)*self.simdt
        self.simt += self.simdt

    def reset(self, simx):
        """ reset simulated system """
        self.simx = simx
        self.simt = 0


class FeedbackController(ControlSystem):
    """ Parent class for feedback control """

    def __init__(self, x):
        ControlSystem.__init__(self, x)

    def feedback(self, x, isStep=False):
        """ return controller for given state """
        if isStep:
            self.step()

        self.x = x
        self.t += self.dt
        return self.u()


class Discrete:
    """general formulation for a discrete system"""
    def __init__(self, x, params=None, n=0):
        self.x = x  # state
        self.n = n  # time step
        self.y = self.output() # output
        self.params = params  # define parameters

    # x^+ = f(t, x, u(t, x))
    def f(self):
        """ define dynamics of discrete function"""
        pass

    def output(self):
        """ set output of the system """
        pass
        
    def step(self):
        """ euler approximation integration"""
        self.x = self.f()
        self.y = self.output()
        self.n += 1

    def run(self, N):
        """ get trajectory from initial value for N steps"""
        temp_x = self.x
        temp_n = self.n
        traj = np.zeros((len(self.x), N))
        for i in range(len(traj.T)):
            traj.T[i] = self.x
            self.step()
        self.x = temp_x
        self.n = temp_n
        self.y = self.output()
        return traj

    def u(self):
        """ input """
        pass


class DiscreteControl(Discrete):
    """ general control affine discrete system"""
    def __init__(self, x, params=None, n=0):
        Discrete.__init__(self, x, params, n)

    # x^+ = f(t, x) + g(t, x)*u(t, x)
    def flow(self):
        """ dynamics of control system """
        return self.f() + np.dot(self.g(), self.u())

    def g(self):
        """ control affine term"""
        pass
