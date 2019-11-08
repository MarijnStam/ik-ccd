# ik-ccd
![Imgur](https://imgur.com/0iVokkO.jpg)
## About
The inverse kinematics problem is an age old problem which many mathematicians have tried to solve. There is no exact solution to this problem, but there are many approximations. Calculation of coordinates when you have a given length of a joint along with an angle is easy, but calculating the needed angle to maneuver this joint to a certain coordinate is not. 

CCD makes an approximation of how you need to rotate the joints of a robot arm to maneuver to a given point in space as quickly as possible. 

This program allows you to configure certain parameters yourself, to get a better understanding of the algorithm.

With the instatiation of our arm class, we can pass along some parameters if desired. If not, the arm will retain default values.

* Pass `l = np.array([l1, l2, l3])` where l1, l2 and l3, are the lengths of the joints to alter the default length values
* Pass `angles = np.array([t1, t2, t3])` where t1, t2 and t3 are the angles of respective joints in radians.
* Alter `newPoint = np.array([x, y])` where x and y are the coordinates of the desired coordinate to maneuver to.

## Installation 
This program can simply be run by executing the plot.py file, only dependencies are numpy and matplotlib and the ccd.py file.
