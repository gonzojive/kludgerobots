#!/usr/bin/env python
import roslib; roslib.load_manifest('hw2')
import rospy
import tf
import math


# class Pose
class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


# class PoseSet
class PoseSet:
    def __init__(self, numPoses = 5000):
        self.numPoses = numPoses
        self.posesPerDim = int(numPoses ** (1.0/3.0) + 0.5)
        self.poses = []

    def initializeUniform(self, xEnds, yEnds, thetaEnds = [0,2*math.pi]):
        denom = 1.0 / self.posesPerDim
        xInterval = float(xEnds[1] - xEnds[0])*denom
        yInterval = float(yEnds[1] - yEnds[0])*denom
        thetaInterval = float(thetaEnds[1] - thetaEnds[0])*denom
        x = xEnds[0]
        for i in range(self.posesPerDim):
            y = yEnds[0]
            for j in range(self.posesPerDim):
                theta = thetaEnds[0]
                for k in range(self.posesPerDim):
                    self.poses.append([x, y, theta])
                    theta += thetaInterval
                y += yInterval
            x += xInterval
