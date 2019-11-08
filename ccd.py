import math
import numpy as np
import scipy.optimize
from matplotlib import pyplot as plt

#   Marijn Stam
#   500761358

#Run plot.py to start the program

class Arm:

    def __init__(self, angles=None, l=None):

        # initial joint angles
        if angles is None: angles = np.array([0.33, 0.3, 0.0])
        self.angles = angles

        # Set the size of the joints, all 100 if none is given
        if l is None: l = np.array([100, 100, 100])
        self.l = l

        #Define the maximum and minimum angles which can be rotated
        self.max_angles = [math.pi/2, math.pi/2, math.pi/2]
        self.min_angles = [0, 0, -math.pi/4]

    #Empty arrays for the x and y points of the arms
    x_points = np.empty(4)
    y_points = np.empty(4)

    def forward_kinematics(self, start_joint):

        #Joint num refers to the current joint which is being calculated for        
        joint_num_x = 0            
        joint_num_y = 0

        #Forward kinematics for the x-coordinates. Start at 0.0 and use trigonometry to calculate the further points based on joint length and its angle
        for joint_num_x in range(start_joint, len(self.x_points)):
            if joint_num_x == 0:
                self.x_points[joint_num_x] = 0
                print("x value at 0 = ", self.x_points[joint_num_x])
            elif joint_num_x == 1:
                self.x_points[joint_num_x] = self.l[0] * np.cos(self.angles[0])
                print("x value at 1 = ", self.x_points[joint_num_x])
            elif joint_num_x == 2:
                self.x_points[joint_num_x] = self.l[1] * np.cos(self.angles[0] + self.angles[1]) + self.x_points[joint_num_x-1]
                print("x value at 2 = ", self.x_points[joint_num_x])
            elif joint_num_x == 3:
                self.x_points[joint_num_x] = self.l[2] * np.cos(np.sum(self.angles)) + self.x_points[joint_num_x-1]
                print("x value at 3 = ", self.x_points[joint_num_x])

        #Same process for y-coordinates
        for joint_num_y in range(start_joint, len(self.y_points)):
            if joint_num_y == 0:
                self.y_points[joint_num_y] = 0
                print("y value at 0 = ", self.y_points[joint_num_y])
            elif joint_num_y == 1:
                self.y_points[joint_num_y] = self.l[0] * np.sin(self.angles[0])
                print("y value at 1 = ", self.y_points[joint_num_y])
            elif joint_num_y == 2:
                self.y_points[joint_num_y] = self.l[1] * np.sin(self.angles[0] + self.angles[1]) + self.y_points[joint_num_y-1]
                print("y value at 2 = ", self.y_points[joint_num_y])
            elif joint_num_y == 3:
                self.y_points[joint_num_y] = self.l[2] * np.sin(np.sum(self.angles)) + self.y_points[joint_num_y-1]
                print("y value at 3 = ", self.y_points[joint_num_y])
        
        

    
    def reverse_kinematics(self, newX, newY):
        
        #Variable decleration, 
        i = 3                                   #Iterator for the IK.
        theta = 0.0                             #Variable for the angle, 0.0 initializes it as a float.
        new_point = np.array([newX, newY])       #Array to hold the desired point.
        dot_product = 0.0                        #Variable for the dot product, 0.0 initializes it as a float.
        threshold = 0.5                         #Minimum distance required from endpoint to desiredpoint, when this is reached the IK is succesful  .              
        
        counter = 0                            
        max_tries = 5000                         #Maximum iterations of CCD allowed
        
        while (np.linalg.norm(new_point - np.array([self.x_points[3], self.y_points[3]])) > threshold) and (counter < max_tries):
            counter += 1

            #Loop from last joint to first
            for i in range(3, 0, -1):                                                                           
                
                #Get the (x, y) for the beginpoint of joint i and the end effector. Substract the endpoint from the beginpoint to get the vector
                end_effector_point = np.array([self.x_points[3], self.y_points[3]])
                joint_begin_point = np.array([self.x_points[i-1], self.y_points[i-1]])
                joint_vector = end_effector_point - joint_begin_point

                #Also get the vector from joint beginpoint to the desired point
                joint_vector_new = new_point - joint_begin_point

                dot_product = np.dot(joint_vector, joint_vector_new)
                cross_product = np.cross(joint_vector, joint_vector_new)

                #Calculate the cosine angle between the two earlier found vectors, this is the angle which we want to rotate the joint.
                cos_angle = dot_product / (np.linalg.norm(joint_vector) * np.linalg.norm(joint_vector_new))

                #Decide if there is enough rotation to be made to prevent endless iterating 
                if cos_angle < 0.9999999:
                    theta = np.arccos(cos_angle)

                    #Check the max/min angle boundaries and decide wether the rotation is CW or CCW.
                    if i != 0:
                        if theta > self.max_angles[i-1]:
                            theta = self.max_angles[i-1]
                        elif theta < self.min_angles[i-1]:
                            theta = self.min_angles[i-1]
                        if cross_product < 0:
                            theta = -theta
  
                    #Apply the rotation to the angle of the respective joint and recalculate (x, y) of all the joints to replot.
                    self.angles[i-1] += theta
                    self.forward_kinematics(i)
                else:
                    print("Not rotating")

        #Is the distance from end effector to desired point within the threshold?
        if np.linalg.norm(new_point - np.array([self.x_points[3], self.y_points[3]])) < threshold:   
            print("IK succesful after iterations needed: ", counter)  
        else:
            print("Could not reach target point within max iterations")  





