import numpy as np
import random
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P


class pendulumDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # initial cart position
            [P.zdot0],  # initial pendulum angle
            [P.theta0],  # initial cart velocity
            [P.thetadot0]   # initial pendulum angular velocity
        ])

        # Mass of the pendulum bob, kg
        self.m = P.m * (1. + alpha * (2. * np.random.rand() - 1.))

        # Mass of the cart, kg
        self.M = P.M * (1. + alpha * (2. * np.random.rand() - 1.))

        # length of the pendulum, m
        self.L = P.L * (1. + alpha * (2. * np.random.rand() - 1.))

        # Damping coefficient, Ns
        self.b = P.b * (1. + alpha * (2. * np.random.rand() - 1.))

        # the gravity constant is well known, so we don't change it.
        self.g = P.g

        # sample rate at which the dynamics are propagated
        self.Ts = P.Ts
        self.force_limit = P.F_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = self.saturate(u, self.force_limit)

        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output

        return y

    def f(self, state, F):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        z = state.item(0)
        zdot = state.item(1)
        theta = state.item(2)
        thetadot = state.item(3)

        sy = np.sin(theta)
        cy = np.cos(theta)
        d = self.m * self.L**2 * (self.M + self.m * (1 - cy**2))

        zddot = (1.0/d)*(-self.m**2*self.L**2*self.g*cy*sy + self.m*self.L**2*(self.m*self.L*thetadot**2*sy - self.b*zdot)) + self.m*self.L*self.L*(1/d)*F;
        thetaddot = (1/d)*((self.m+self.M)*self.m*self.g*self.L*sy - self.m*self.L*cy*(self.m*self.L*thetadot**2*sy - self.b*zdot)) - self.m*self.L*cy*(1/d)*F;

        xdot = np.array([[zdot],
                         [zddot],
                         [thetadot],
                         [thetaddot]])

        return xdot

    def h(self):
        # return the output equations
        # could also use input u if needed
        z = self.state.item(0)
        theta = self.state.item(2)
        y = np.array([[z],
                      [theta]])

        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit * np.sign(u)

        return u
