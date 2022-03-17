import numpy as np
import pendulumParam as P


class pendulumController:
    def __init__(self):

        self.error_d1 = 0.0
        self.K = P.K  # state feedback gain
        x = np.array([
            [P.z0],  # initial cart position
            [P.zdot0],  # initial pendulum angle
            [P.theta0],  # initial cart velocity
            [P.thetadot0]   # initial pendulum angular velocity
        ])

        self.ki = P.ki
        self.integrator = self.K @ x / (-self.ki)
        self.limit = P.F_max  # max force
        self.Ts = P.Ts  # sample rate of the controller

    def update(self, x_r, x):
        state_error = x - x_r

        F_unsat = -self.K @ state_error
        F_sat = self.saturate(F_unsat)

        return F_sat.item(0)

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit * np.sign(u)
        return u
