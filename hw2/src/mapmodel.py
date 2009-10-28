import rospy
import tf
import threading
from vector import *
import nav_msgs
import nav_msgs.msg
import geometry_msgs

def mapFloatIntoDiscretizedBucket(f, minFloat, maxFloat, numBuckets):
    # f prefix float i discrete
    fSizeOfBucket = float(maxFloat - minFloat) / float(numBuckets)
    iBucket = int( float(f - minFloat) / fSizeOfBucket)
    if iBucket < 0:
        return 0
    elif iBucket >= numBuckets:
        return numBuckets - 1
    else:
        return iBucket


class MapModel:
    def __init__(self):
        self._initialized = False

        
        self.fHeight = 10.0
        self.fWidth = 10.0
        self.xMax = 0.0
        self.xMin = 0.0
        self.yMin = 0.0
        self.yMin = 0.0

        def mapCallback(mapOccGrid):
            self.mapMetaData = mapOccGrid.info
            self.grid = mapOccGrid.data
            self._annotateMapMetaData()
            self._initialized = True

        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, mapCallback) # listen to "laser"

    def _annotateMapMetaData(self):
        meta = self.mapMetaData

        # FIXME: account for the orientation of the map
        resolution = meta.resolution
        self.fHeight = resolution * float(meta.height)
        self.fWidth = resolution * float(meta.height)
        
        self.xMin = meta.origin.position.x
        self.xMax = self.xMin + self.fWidth
        
        self.yMin = meta.origin.position.y
        self.yMax= self.yMin + self.fHeight


    def initializedp(self):
        return self._initialized

    # given a pose (relative to the map), return an artificial laser scan from the post
    # location and orientation.  Casts rays appropriately 
    def laserScanAtPose(self, pose):
        # pseudocode:
        # for each potential ray in the laser
        # let V = the the vector from the pose origin 12m away in the direction
        # of the laser scan
        # cast that ray in the map and add it to the distances of the laser scan
        return None

    # returns whether the map at the given point is occupied
    def probeAtPoint(self, pt):
        # here we the
        meta = self.mapMetaData
        xDiscrete = mapFloatIntoDiscretizedBucket(pt[0],  self.xMin, self.xMax, meta.width)
        yDiscrete = mapFloatIntoDiscretizedBucket(pt[1],  self.yMin, self.yMax, meta.height)
        # Given an X, Y coordinate, the map is access via data[Y*meta.width + X]
        # the grid has values between 0 and 100, and -1 for unknown
        probabilityOfOccupancy = self.grid[yDiscrete * self.width + xDiscrete]
        if probabilityOfOccupancy < 0:
           probabilityOfOccupancy = 100

        return float(probabilityOfOccupancy) / 100.0

    # returns the maximum distance from vSTart to vEnd that can be gone in the map
    # before bumping into an obstacle
    def castVector(self, vStart, vEnd):
        # v is the vector to cast relative to vStart
        v = vector_minus(vEnd, vStart)
        vMagnitude = vector_length(v)
        # v normalized
        vUnit = vector_normalize(v)
        # samples are taken every sampleSize meters along the way
        # from vStart to vEnd
        sampleSize = .10 # 10 cm at a time

        scale = 0
        while scale <= vMagnitude:
            # vUnit * scale + vStart = probe point
            probePt = vector_add(vStart, vector_scale(vUnit, scale))
            if self.probeAtPoint(probePt):
                return scale
            scale += sampleSize

        return vMagnitude
