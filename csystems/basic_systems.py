'''
Class defintions for creating basic feedback control affine systems

'''
import numpy as np


class ControlSystem:
    '''
    general control affine system
    '''
    def __init__(self, x, dt=.1, params=None,  t=0):
        self.x = x  # state
        self.t = t  # initial time
        self.dt = dt  # time step
        self.params = params  # define parameters

    # dynamics of control system
    def flow(self):
        # x dot = f(x) + g(x)*u
        return self.f() + self.g() @ self.u()

    # drift
    def f(self):
        raise NotImplementedError

    # affine term
    def g(self):
        raise NotImplementedError

    # input
    def u(self):
        raise NotImplementedError

    # euler approximation
    def step(self):
        self.x += self.flow()*self.dt
        self.t += self.dt

    # RK 4th order approx
    def runge_kutta_step(self):
        state = np.copy(self.x)
        k1 = self.flow()*self.dt
        self.x = state + .5*k1
        k2 = self.flow()*self.dt
        self.x = state + .5*k2
        k3 = self.flow()*self.dt
        self.x = state + k3
        k4 = self.flow()*self.dt
        self.x = state + (k1+2*k2+2*k3+k4)/6

    # get trajectory from initial value
    def run(self, t):
        x_i = self.x
        traj = n.zeros((len(self.x), int(t/self.dt)))
        for i in range(len(traj.T)):
            traj.T[i] = self.x
            self.step()
        self.x = x_i
        return traj


class SingleIntegrator(ControlSystem):
    '''
    single integrator control system with x, y
    '''
    def __init__(self, x):
        ControlSystem.__init__(self, x)

    # single integrator dynamics
    def f(self):
        xdot = 0
        ydot = 0
        return np.array([xdot, ydot]).T

    def g(self):
        return np.identity(2)

    # 2 inputs: v_x , v_y
    def u(self):
        return np.array([0, 0])


class DoubleIntegrator(ControlSystem):
    '''
    double integrator control system with x, y, vx, and vy
    '''
    def __init__(self, x):
        ControlSystem.__init__(self, x)

    # double integrator dynamics
    def f(self):
        xdot = self.x[2]
        ydot = self.x[3]
        vxdot = 0
        vydot = 0
        return np.array([xdot, ydot, vxdot, vydot]).T

    def g(self):
        return np.array([[0, 0, 1, 0], [0, 0, 0, 1]]).T

    # 2 inputs: acc_x , acc_y
    def u(self):
        return np.array([0, 0])


class SingleUnicycle(ControlSystem):
    '''
    unicycle control system with x, y, th
    '''
    def __init__(self, x):
        ControlSystem.__init__(self, x)

    # single unicycle dynamics
    def f(self):
        return np.array([0, 0, 0]).T

    def g(self):
        return np.array([[np.cos(self.x[2]), np.sin(self.x[2]), 0], [0, 0, 1]]).T

    # wrap theta
    def step(self):
        ControlSystem.step(self)
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

    # 2 inputs: velocity, angular velocity
    def u(self):
        return np.array([0, 0])


class DoubleUnicycle(ControlSystem):
    '''
    unicycle control system with x, y, v, th
    '''
    def __init__(self, x):
        ControlSystem.__init__(self, x)

    # double unicycle dynamics
    def f(self):
        xdot = self.x[2]*np.cos(self.x[3])
        ydot = self.x[2]*np.sin(self.x[3])
        vdot = 0
        thdot = 0
        return np.array([xdot, ydot, vdot, thdot]).T

    def g(self):
        return np.array([[0, 0, 1, 0], [0, 0, 0, 1]]).T

    # wrap theta
    def step(self):
        ControlSystem.step(self)
        self.x[3] = np.arctan2(np.sin(self.x[3]), np.cos(self.x[3]))

    # 2 inputs: acceleration, angular velocity
    def u(self):
        return np.array([0, 0])


class NetworkSystem(ControlSystem):
    '''
    Parent class for systems with interactions
    '''
    def __init__(self, x, sys_list):
        ControlSystem.__init__(self, x)
        self.sys_list = sys_list


class DrawSystem(ControlSystem):
    '''
    Parent class for drawing simulations
    '''
    def __init__(self, x):
        ControlSystem.__init__(self, x)
        self.drawings = []

    # initiliaze drawings
    def draw_setup(self, axes=None):
        raise NotImplementedError

    # update drawings
    def draw_update(self, axes=None):
        raise NotImplementedError


class SimulationSystem(ControlSystem):
    '''
    Parent class for generating traces for desired input
    '''
    def __init__(self, x, gamma, simx=None):
        ControlSystem.__init__(self, x)
        self.gamma = gamma  # fake feedback controller
        self.simx = simx  # simulated state
        self.simt = 0  # simulated time
        self.simdt = self.dt  # simulated dt

    # xdot for a given state
    def gamma_flow(self, x):
        tempx = self.x.copy()
        self.x = x
        xdot = self.f() + self.g() @ self.gamma(x)
        self.x = tempx
        return xdot

    # flow for simx using gamma
    def gamma_step(self):
        self.simx += self.gamma_flow(self.simx)*self.simdt
        self.simt += self.simdt

    # reset simulated system
    def reset(self, simx):
        self.simx = simx
        self.simt = 0


class FeedbackController(ControlSystem):
    '''
    Parent class for feedback control
    '''
    def __init__(self, x):
        ControlSystem.__init__(self, x)

    # return controller for given state
    def feedback(self, x):
        self.x = x
        self.t += dt
        return self.u()
