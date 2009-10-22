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




# LaserInterpreter
#   This has been simplified for assignment 2, now it just stores the latest reading
#   Later on, we can add functions to use the latest reading as described in the assignment
class LaserInterpreter:
    def __init__(self): # constructor
        self.maxRadius = 11.0  # laser only accurate to 12 feet, let's say 11 to be safe
        self.maxTheta = math.pi  # theta in [0, pi]
        self.latestReading = None

    # this is what the update loop calls in LaserInterpreter
    def laserReadingNew(self, reading):
        # Do some processing on the new laser reading
        self.latestReading = reading

    # casts a vector from the origin of the robot's laser.  We basically find
    # the angle and then call castRayPolar
    def castVector(self, vector):
        angle = 0
        if abs(vector[0]) < .01 and abs(vector[1]) < .01:
            angle = 0
        else:
            angle = normalizeAngle360(vector_angle_general([0.0,-1.0], vector))
        if angle > math.pi:
            angle = math.pi
        return self.castRayPolar(angle)

    # casts a ray in the given polar direction and returns how far away an object is
    # in that direction
    def castRayPolar(self, theta):
        if not self.latestReading:
            return 0
        theta = normalizeAngle360(theta)
        numBuckets = len(self.latestReading.ranges)
        radiansPerLaserReadingRange = pi / float(numBuckets)
        # figure out which bucket it would be in if the readings were stored right to left
        bucketRightToLeft = int(theta / radiansPerLaserReadingRange)
        # now reverse that
        bucketLeftToRight = numBuckets - bucketRightToLeft
        # restrict to range
        if bucketLeftToRight >= numBuckets:
            bucketLeftToRight = numBuckets - 1
        elif bucketLeftToRight < 0:
            bucketLeftToRight = 0
        # return that range.  simple and stupid
        rng = self.latestReading.ranges[bucketLeftToRight]
        #rospy.loginfo("For angle %0.2f bucket %d range %0.2f", r2d(theta), bucketLeftToRight, rng)
        return rng
        
    def logReadingInfo(self, reading):
        rospy.loginfo("Min: %d  Max: %d  Inc: %f  Len: %d", r2d*reading.angle_min, r2d*reading.angle_max, r2d*reading.angle_increment, len(reading.ranges))
        print reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0]



# assuming the laser is 180 degrees, returns what angle the ith laser reading is
# given an array of laser ranges (which are arranged left to right)
def laserRangeAngle(i, readingRanges):
    num_scan_points = len(readingRanges)
    if num_scan_points > 0:
        return (1.0 - float(i) / float(num_scan_points)) * pi
    else:
        return 0 #degenerate
        
# Given a laser reading, returns a bunch of cartesian points.
# to the left is positive y.  straight is positive x
def laserReadingToCartesianPoints(reading, position):
    pos = position.position()
    local = map(lambda rng,i: polarToCartesian(rng, laserRangeAngle(i, reading.ranges)+pos[1]), reading.ranges, xrange(0, len(reading.ranges)))
    return [[p[0]-pos[0][0],p[1]-pos[0][1]] for p in local]

