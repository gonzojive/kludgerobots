import math
from math import *
 
# Given an angle and a radius, returns the X,Y coordinates of that point in cartesian coordinates
# this assumes the following:
# 1. The cartesian coordinate is such that in front of the robot is positive X and the LEFT of
# the robot is positive Y
# 2. theta = 0, r = 1 corresponds to (0, -1) and theta = pi/2, r=1 corresponds to (1, 0)
# i.e. to the left is positive y. straight is positive x
def polarToCartesian(r, theta):
  x = r * cos(theta)
  y = r * sin(theta)
  return [y, -x]
 
def radiansToDegrees(rads):
    return rads * 180.0/math.pi
 
NUMERIC_TOL = .000001

def d2r(x):
    return x*3.14159/180.0

def r2d(x):
    return x*180.0/3.14159
 
def normalizeAngle360(rads):
    
    if (rads > 2.0 * pi + NUMERIC_TOL):
        return normalizeAngle360(rads - (2.0 * pi))
 
    elif (rads < 0 - NUMERIC_TOL):
        return normalizeAngle360(rads + (2.0 * pi))
 
    else:
        return rads
 
# ensures a positive angle between 0 and 2pi
def normalizeAngle180(rads):
    rads = normalizeAngle360(rads)
    if (rads > pi):
        return 2 * pi - rads
    else:
        return rads
 
def normalizeAngle90(rads):
    rads = normalizeAngle360(rads)
    # > 180 degrees, reduce to the 180 sphere
    if (rads > pi + NUMERIC_TOL):
        return normalizeAngle90(2 * pi - rads)
    elif (rads > pi): # within tolerance
        return 0
    elif (rads > pi/2.0):
        return pi - rads
    else:
        return rads
