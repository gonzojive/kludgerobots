import roslib;
import rospy
import tf
from math import *
from line import *
from util import *
from vector import *
from lineviz import *
from walls import *
from quaternion import *

class RedGlobalCompass:
    def __init__(self):
        self.walls = []

class RobotPosition:
    def __init__(self):
        self.initialized = False  # can't start until we initialize odometry
        self.odomTrans = [0,0]  
        self.odomRot = 0
        self.mapTrans = [0,0]
        self.mapRot = 0
    def resetOdom(self, t, r):  # resets the odometry offsets to the current odometry value
        self.odomTrans0 = t
        self.odomTrans =  [0.0, 0.0]
        
        # incorrect: self.odomRot0 = math.acos(r[3])*2.0 # set the original rotation
        self.odomRot0 = self.odomRot = quatToAngleAboutPositiveZ(r)
        rospy.loginfo("Original ODOM: (%0.2f, %0.2f) at %0.2f degrees", self.odomTrans0[0], self.odomTrans0[1], self.odomRot0)
    def odomReadingNew(self, t, r):  # calculate a new odometry reading with respect to the offsets
        self.odomTrans[0] = t[0] - self.odomTrans0[0]
        self.odomTrans[1] = t[1] - self.odomTrans0[1]
        self.odomRot = quatToAngleAboutPositiveZ(r)
        rospy.loginfo("ODOM Quaternion: (%0.2f, %0.2f, %0.2f, %0.2f) => theta = %0.2f degrees // odomRot = %0.2f", r[0], r[1], r[2], r[3], r2d(self.theta()), r2d(self.odomRot))
        #self.odomRot = (math.acos(r[3])*2.0 - self.odomRot0)
        self.logPosInfo()
    def mapPositionNew(self, mapT, mapR):  # take in a new map reading
        4
        #self.mapTrans = mapT  # assume map tranlation and rotation are base truth
        #self.mapRot = mapR
        #self.resetOdom(self.odomTrans, self.odomRot)  # set odometry offsets back to 0
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
        #return relativeToOdometryFrameRotation
        return relativeToOriginalRotation
        #return self.mapRot+self.odomRot
    # returns the vector going forward out of the robot
    def forwardVector(self):
        return polarToCartesian(1.0, self.theta() + pi/2.0)

    #    def localToGlobal(self, pt):

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
        rospy.loginfo("Odometry: (%0.2f, %0.2f) at xxx degrees", self.odomTrans[0], self.odomTrans[1])

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

# this class is responsible for taking laser readings and updating the robot position (global compass)
# it also has methods for figuring out where obstacles are.  This is called by the MoveToGoal object
class LaserInterpreter:
    def __init__(self, position): # constructor
        self.position = position
        self.maxRadius = 11.0  # laser only accurate to 12 feet, let's say 11 to be safe
        self.maxTheta = math.pi  # theta in [0, pi]
        self.mapviz = LocalMapVisualizer()
        self.latestReading = None

    def findWallsAndUpdateGlobalCompass(self,reading):
        # do nothing heren
        #rospy.loginfo('TODO find walls and update the global compass')
        4 + 4

    # this is what the update loop calls in LaserInterpreter.  From here we update
    # the global position
    def laserReadingNew(self, reading):
        # Do some processing on the new laser reading
        self.latestReading = reading
        # update the global compass!
        self.findWallsAndUpdateGlobalCompass(reading)

    # casts a vector from the origin of the robot's laser.  We basically find
    # the angle and then call castRayPolar
    def castVector(self, vector):
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
        
#  def ransac(self, reading):
#    cartesianPoints = laserReadingToCartesianPoints(reading, self.position)
#    [bestLine, inliers, extremes] = fitLineWithRansac(cartesianPoints, .03)
##    rospy.loginfo("Extremes: %0.2f %0.2f %0.2f %0.2f", extremes[0], extremes[1], extremes[2], extremes[3])
#    self.mapviz.vizPoints(inliers)
#    #for pt in cartesianPoints:
#    #rospy.loginfo("(%0.2f, %0.2f)", pt[0], pt[1])
#    def s(arr):
#      return "[%0.2f, %0.2f]" % (arr[0], arr[1])
#    rospy.loginfo("Line: %s trajectory: %s" % (s(bestLine.origin),  s(bestLine.trajectory)))
##    rospy.loginfo("  %i Inliers of %i readings: %s" % (len(inliers), len(reading.ranges), map(s, inliers)))
#    angleDiff = self.localMap.wallIs(bestLine, extremes)
#    if angleDiff != None:
#      self.position.addMapRotationOffset(angleDiff)

