 # THINGS TO DO
# Change data plotter to hold the the new state variables, not just z position but a z and theta
# Have animation draw new box

import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from pendulumController import pendulumController

# instantiate reference input classes
reference = signalGenerator(amplitude=5.0, frequency=0.02, y_offset=0.0)
force = signalGenerator(amplitude=0.0, frequency=0.001)
pendulum = pendulumDynamics(alpha=0.0)
controller = pendulumController()

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # Propate dynamics at rate Ts
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        r = P.des_state
        r[0] = reference.square(t)
        n = 0.0  # noise.random(t)  # simulate sensor noise
        x = pendulum.state
        u = controller.update(r, x)
        y = pendulum.update(u)
        t = t + P.Ts

    # update animation
    animation.update(pendulum.state)
    dataPlot.update(t, r.item(0), pendulum.state, u)

    #plt.show()
    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
