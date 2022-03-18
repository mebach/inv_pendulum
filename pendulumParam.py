# Inverted Pendulum Parameter File
import numpy as np
import control

# Desired state, of the form (z, zdot, theta, thetadot)
des_state = np.array([[0.0],
                      [0.0],
                      [np.pi],
                      [0.0]])

# Physical parameters of the mass known to the controller
m = 1.0  # mass of pendulum bob kg
M = 5.0  # mass of cart kg
L = 2.0  # length of pendulum, m
b = 1.0 # damping coefficient Kg/s
g = -10  # gravity m/s^2

# parameters for animation
length = 5.0
width = 1.0
cart_length = 1.0
cart_height = 0.3
pendulum_length = L
wheel_radius = 0.1
pendulum_bob_radius = 0.2

# Initial Conditions
z0 = 0.0  # initial position of cart
zdot0 = 0.0  # initial velocity of cart m/s
theta0 = np.pi + 0.01  # initial angle of pendulum
thetadot0 = 0.0  # initial angular velocity of pendulum

# Controller paramters
ki = 1.0


# Simulation Parameters
t_start = 0  # Start time of simulation
t_end = 100.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

# dirty derivative parameters
sigma = 0.05 # cutoff freq for dirty derivative
beta = 0.05  # dirty derivative gain

# saturation limits
F_max = 1000.0  # Max force, N

####################################################
#                 State Space
####################################################

s = 1

# State space representation of linearized system:
A = np.array([[0, 1, 0, 0],
     [0, -b/M, -m*g/M, 0],
     [0, 0, 0, 1],
     [0, -s*b/(M*L), -s*(M+m)*g/(M*L), 0]])

B = np.array([[0],
     [1/M],
     [0],
     [s*1/(M*L)]])

# find the eigenvalues w and eigenvectors v:
w, v = np.linalg.eig(A)
# print(w)

# Is this system controllable? The rank of the controllability matrix should be full, in this case, 4
# print(np.linalg.matrix_rank(cnt.ctrb(A, B)))

# If the system is indeed controllable, then there exists a matrix K such that a control law can be used using the
# full state x, i.e. u = -Kx We can force the closed-loop system to have any desired eigenvalues (stable eigenvalues
# by choosing an appropriate K matrix. This is done by using the pole function in the control library

# Mostly arbitrary eigenvalues, but they are all negative to make the system "stable". Remember that from pole graphs, poles on the left side are stable and positive ones are unstable.
des_eigs = [-32.42, -0.75 + 0.79j, -0.75-0.79j, -.82]
K = control.place(A, B, des_eigs)

# print(K)
# print(np.linalg.eig(A-B*K))  # note that the eigenvalues returned by this are the same as des_eigs

#################################################
#                   LQR                         #
#################################################

# Q = np.array([[1, 0, 0, 0],
#      [0, 1, 0, 0],
#      [0, 0, 10, 0],
#      [0, 0, 0, 100]])
#
# R = np.array([[0.001]])
#
# K, _, __ = control.lqr(A, B, Q, R)






