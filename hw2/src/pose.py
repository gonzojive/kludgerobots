#!/usr/bin/env python
#import roslib; roslib.load_manifest('hw2')
#import rospy
#import tf
import math


# class Pose
#   contains the robot pose info (x position, y position, theta) and a weight
class Pose:
    def __init__(self, x, y, theta, weight = 1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


# class PoseSet
#   contains the list of poses
class PoseSet:
    # __init__
    # parameter: numPoses = the number of poses, either a list or a total value
    def __init__(self, numPoses = [20, 20, 10]):    # default to 20 in x and y, 10 in theta
        if not isinstance(numPoses, list):  # if the argument is not a list (it's a number)
            posesPerDim = int(numPoses ** (1.0/3.0) + 0.5)  # nearest perfect cube
            self.numPoses = [posesPerDim, posesPerDim, posesPerDim] # same poses in each dim
        else:   # otherwise, it's a list, just use the values directly
            self.numPoses = numPoses
        self.poses = []

    # initializeUniform: sets a uniform distribution of poses in a given range
    # parameters:
    #   xEnds   --  an [xmin, xmax] list
    #   yEnds   --  a [ymin, ymax] list
    #   thetaEnds   --  a [thetamin, thetamax] list, defaults to [0, 2pi]
    # note: as it's written, the distribution includes the start pose but not the end pose
    # this is to avoid double-counting 0 and 2pi
    def initializeUniform(self, xEnds, yEnds, thetaEnds = [0,2*math.pi]):
        xInterval = float(xEnds[1] - xEnds[0]) / self.numPoses[0]
        yInterval = float(yEnds[1] - yEnds[0]) / self.numPoses[1]
        thetaInterval = float(thetaEnds[1] - thetaEnds[0]) / self.numPoses[2]
        x = xEnds[0]
        for i in range(self.numPoses[0]):
            y = yEnds[0]
            for j in range(self.numPoses[1]):
                theta = thetaEnds[0]
                for k in range(self.numPoses[2]):
                    self.poses.append([x, y, theta])
                    theta += thetaInterval
                y += yInterval
            x += xInterval