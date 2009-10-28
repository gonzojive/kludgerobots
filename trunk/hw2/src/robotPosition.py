import roslib
import rospy
import tf
from math import *
from util import *
from vector import *
from quaternion import *

class RobotPosition:
    def __init__(self):
        self.initialized = False  # can't start until we initialize odometry
        self.odomTrans = [0,0]  
        self.odomRot = 0
        self.mapTrans = [0,0]
        self.mapRot = 0
        self.previous = None
    def resetOdom(self, t, r):  # resets the odometry offsets to the current odometry value
        self.odomTrans0 = t
        self.odomTrans =  [0.0, 0.0]
        self.odomRot0 = self.odomRot = quatToAngleAboutPositiveZ(r)
        rospy.loginfo("Original ODOM: (%0.2f, %0.2f) at %0.2f degrees", self.odomTrans0[0], self.odomTrans0[1], self.odomRot0)
    def odomReadingNew(self, t, r):  # calculate a new odometry reading with respect to the offsets
        self.previous = self.odomTrans + [self.odomRot]
        self.odomTrans[0] = t[0] - self.odomTrans0[0]
        self.odomTrans[1] = t[1] - self.odomTrans0[1]
        self.odomRot = quatToAngleAboutPositiveZ(r)
        if self.previous[0] != self.odomTrans[0] or self.previous[1] != self.odomTrans[1] or self.previous[2] != self.odomRot:
            self.logPosInfo()
    def addMapRotationOffset(self, offset):  # just offset the current reading
        self.mapRot += offset

    # HEY!  This is the important interface for reading out values.
    # returns the most recent map position + any more recent odometry offsets
    # the x, y, and theta
    def position(self):  
        return [self.origin(), self.theta()]

    def theta(self):
        # relativeToOriginalRotation corresponds to the angle since this node started up 
        relativeToOriginalRotation = normalizeAngle360(self.odomRot - self.odomRot0)
        relativeToOdometryFrameRotation = normalizeAngle360(self.odomRot)
        return relativeToOriginalRotation

    # returns the vector going forward out of the robot
    def forwardVector(self):
        return polarToCartesian(1.0, self.theta() + pi/2.0)

    # given some point relative to the robot's position when we first started listening in,
    # returns a point relative to the current robot position
    def globalToLocal(self, pt):
        vToPtGlobal = vector_minus(pt, self.origin())
        rotated_vToGoal = vector_rotate_2d( vToPtGlobal, -1.0 * self.theta())
        return rotated_vToGoal

    def origin(self):
        x = self.mapTrans[0]+self.odomTrans[0]
        y = self.mapTrans[1]+self.odomTrans[1]
        return vector_rotate_2d([x, y] , -1.0 * self.odomRot0)

    def logPosInfo(self):
        rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f degrees", self.odomTrans[0], self.odomTrans[1], r2d(self.theta()))
