import roslib
import rospy
import tf
from math import *
from util import *
from vector import *
from quaternion import *
from sensor_msgs.msg import LaserScan
import threading

# Laser
#   This has been simplified for assignment 2, now it just stores the latest reading
#   Later on, we can add functions to use the latest reading as described in the assignment
class Laser:
    def __init__(self): # constructor
        self.maxRadius = 11.0  # laser only accurate to 12 feet, let's say 11 to be safe
        self.maxTheta = math.pi  # theta in [0, pi]
        self.latestReading = None
        rospy.Subscriber("laser", LaserScan, self.laserReadingNew)
        self.readingLock = threading.Lock()

    # callback function to get the newest reading
    def laserReadingNew(self, reading):
        self.readingLock.acquire()  # <--- grab the lock --->
        self.latestReading = reading
        self.readingLock.release()  # <--- release the lock --->

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
def laserRangeAngle(i, numReadings):
    i = numReadings - i - 1
    if numReadings > 0:
        return (1.0 - float(i) / float(numReadings)) * pi
    else:
        return 0 #degenerate
        


def laserScanToVectors(laserScan, increment = 8):
    # filter out the maxrange stuff
    numReadings = len(laserScan)
    ranges = filter(lambda r: r[0] < 11.5, laserScan)

    # take only occasional range because we don't want to do too much computation and
    # also don't want to take too big of an exponent
    if increment != 1:
        temp = laserScan[0:-1:increment]
        ranges = temp
    
    result = map(lambda rng: polarToCartesian(rng[0], laserRangeAngle(rng[1], numReadings)), ranges)
    return result

        
    
