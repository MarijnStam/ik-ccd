import numpy as np
import time
from matplotlib import pyplot as plt

import ccd

#   Marijn Stam
#   500761358

def plot():
    """A function for plotting an arm, and having it calculate the
    inverse kinematics such that given the mouse (x, y) position it
    finds the appropriate joint angles to reach that point."""

    #Create an instance of the arm
    #You can pass along self-defined lengths and angles if you wish, also possible to pass nothing. Arm will initialize with default values.
    robot_arm = ccd.Arm(l = np.array([75, 75, 75]))

    #Calculate all the x and y points for the joints in the arm
    #We pass 0 to start at the basepoint of the arm
    robot_arm.forward_kinematics(0)

    #Define your desired endpoint of the end effector
    newPoint = np.array([100, 100])

    #Plot the initialized arm along with the desired endpoint
    plt.plot(robot_arm.x_points, robot_arm.y_points)
    plt.plot(newPoint[0], newPoint[1],'ro')

    #Run the CCD algorithm
    robot_arm.reverse_kinematics(newPoint[0], newPoint[1])
    
    #Plot the arm again with the new positions
    plt.plot(robot_arm.x_points, robot_arm.y_points)
    plt.show()

plot()