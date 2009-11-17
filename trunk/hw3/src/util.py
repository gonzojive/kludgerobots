import math
from math import *
NUMERIC_TOL = .000001

# Given an angle and a radius, returns the X,Y coordinates of that point in cartesian coordinates
# this assumes the following:
# 1. The cartesian coordinate is such that in front of the robot is positive X and the LEFT of
#    the robot is positive Y
# 2. theta = 0, r = 1 corresponds to (0, -1) and theta = math.pi/2, r=1 corresponds to (1, 0)
# i.e. to the left is positive y.  straight is positive x
def polarToCartesian(r, theta):
  x = r * cos(theta)
  y = r * sin(theta)
  return [y, -x]

def radiansToDegrees(rads):
    return rads * 180.0/math.pi

def r2d(rads):
    return rads * 180.0/math.pi

def d2r(rads):
    return rads * math.pi / 180.0


def normalizeAngle360(rads):
    if (rads > 2.0 * math.pi + NUMERIC_TOL):
        return normalizeAngle360(rads - (2.0 * math.pi))

    elif (rads < 0 - NUMERIC_TOL):
        return normalizeAngle360(rads + (2.0 * math.pi))

    else:
        return rads

# ensures a positive angle between 0 and math.pi 
def normalizeAngle180(rads):
    rads = normalizeAngle360(rads)    
    if (rads > math.pi):
        return 2 * math.pi - rads
    else:
        return rads

# ensures a positive angle between 0 and math.pi 
def normalizeAnglePosNeg180(rads):
  rads360 = normalizeAngle360(rads)
  if (rads > math.pi):
    return -1.0 * normalizeAngle180(rads)
  else:
    return rads

def normalizeAngle90(rads):
    rads = normalizeAngle360(rads)
    # > 180 degrees, reduce to the 180 sphere
    if (rads >  math.pi + NUMERIC_TOL):
        return normalizeAngle90(2 * math.pi - rads)
    elif (rads >  math.pi): # within tolerance
        return 0
    elif (rads > math.pi/2.0):
        return math.pi - rads
    else:
        return rads

def smallestAngleDifference(a1, a2):
    d1 = normalizeAngle360(a1 - a2)
    d2 = normalizeAngle360(a2 - a1)
    return min(d1, d2)

def close(a, b, maxDist = 0.01):
    absDiff = [abs(v[1]-v[0]) for v in zip(a, b)]
    return len(filter(lambda x: x > maxDist, absDiff)) == 0

def closeToOne(a, bList, maxDist = 0.01):
    for b in bList:
        if close(a, b, maxDist):
            return True
    return False
    
    
