import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import sys
sys.path.append('..')  # add parent directory
import numpy as np
import pendulumParam as P


class pendulumAnimation:
    '''
        Create mass animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.length = P.length
        self.width = P.width
        plt.axis([-P.length-P.length/5, 2*P.length, -P.length, 2*P.length]) # Change the x,y axis limits
        plt.plot([-P.length-P.length/5, 2*P.length], [-P.wheel_radius, -P.wheel_radius], 'k--')    # Draw track
        # plt.plot([-P.length, -P.length], [0, 2*P.width], 'k')  # Draw wall

        # Draw mass is the main function that will call the functions:
    def update(self, u):
        # Process inputs to function
        z = u.item(0)   # position of cart
        theta = u.item(2)  # angle of the pendulum

        self.drawCart(z)
        self.drawPendulum(z, theta)

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawCart(self, z):
        x = z  # x coordinate
        y = 0.0  # y coordinate
        xy = (x, y)
        xy2 = (x+P.cart_length, y)  # Bottom right corner of rectangle

        # When the class is initialized, a Rectangle patch object will be
        # created and added to the axes. After initialization, the Rectangle
        # patch object will only be updated.
        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Rectangle(xy, P.cart_length, P.cart_height, fc='blue', ec='black'))
            self.ax.add_patch(self.handle[0])  # Add the patch to the axes
            self.handle.append(mpatches.CirclePolygon(xy, radius=P.wheel_radius, fc='red', ec='black'))
            self.ax.add_patch(self.handle[1])
            self.handle.append(mpatches.CirclePolygon(xy2, radius=P.wheel_radius, fc='red', ec='black'))
            self.ax.add_patch(self.handle[2])
        else:
            self.handle[0].set_xy(xy)         # Update patch
            self.handle[1]._xy = xy
            self.handle[2]._xy = xy2

    def drawPendulum(self, z, theta):
        X = [z+P.cart_length/2, z+P.cart_length/2+np.sin(theta)*P.pendulum_length]  # X data points
        Y = [P.cart_height, P.cart_height - np.cos(theta) * P.pendulum_length]

        xy = (X[1], Y[1])
        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=2, c='black')
            self.handle.append(line)
            self.handle.append(mpatches.CirclePolygon(xy, radius=P.pendulum_bob_radius, fc='green', ec='black'))
            self.ax.add_patch(self.handle[4])
        else:
            self.handle[3].set_xdata(X)  # Update the line
            self.handle[3].set_ydata(Y)
            self.handle[4]._xy = xy

