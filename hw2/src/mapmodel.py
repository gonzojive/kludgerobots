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

        # parameters used to discretize floating point map coords into buckets
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
            rospy.loginfo("Computing obstacle distance grid: %i x %i grid", self.mapMetaData.width, self.mapMetaData.height)
            
            #self.dgrid = self.computeDistanceFromObstacleGrid()
            rospy.loginfo("Initialized Map completedly")
            self._initialized = True

        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, mapCallback) # listen to "laser"

    # computes some initial values
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

    def computeDistanceFromObstacleGrid(self):
        # create a row-major grid that holds the nearest obstacle from
        # each point in the original map grid

        def discreteGridRefToReal(xDiscrete, yDiscrete):
            res = self.mapMetaData.resolution
            return [float(xDiscrete) * res + self.xMin, float(yDiscrete) * res + self.yMin]

        # the grid has a value between 0 and 100 that indicates the
        # probability that an element of the grid is occupied.  We
        # assume anything with a value >= 10 is occupied.  If the
        # value is -1 it means it is unknown and we assume that it is
        # an obstacle
        def obstaclepFromProbability(p):
            if p >= 10 or p == -1:
                return True
            else:
                return False

        boolgrid = map(obstaclepFromProbability, self.grid)
        distgrid = [ 10.0 for x in boolgrid]

        # given discrete grid indexes returns these functions 
        def gridValueAtDiscreteCoordinate(grid, xDiscrete, yDiscrete):
            return grid[yDiscrete * self.mapMetaData.width + xDiscrete]

        def setGridValueAtDiscreteCoordinate(grid, xDiscrete, yDiscrete, value):
            grid[yDiscrete * self.mapMetaData.width + xDiscrete] = value


        # gotta love O(n^4)
        for xDiscrete in xrange(0, self.mapMetaData.width):
            for yDiscrete in xrange(0, self.mapMetaData.height):
                bestDistanceSquared = None

                fpoint = discreteGridRefToReal(xDiscrete, yDiscrete)
                
                for xDiscreteCandidate in xrange(0, self.mapMetaData.width):
                    for yDiscreteCandidate in xrange(0, self.mapMetaData.height):
                        obstaclep = gridValueAtDiscreteCoordinate(boolgrid, xDiscreteCandidate, yDiscreteCandidate)
                        if obstaclep:
                            # compute distance to obstacle
                            fcandidate = discreteGridRefToReal( xDiscreteCandidate, yDiscreteCandidate)
                            candidateDistanceSquared = vector_length_squared(vector_minus(fpoint, fcandidate))
                            # see if it's the best found so far
                            if not bestDistanceSquared or candidateDistanceSquared < bestDistanceSquared:
                                bestDistanceSquared = candidateDistanceSquared
                setGridValueAtDiscreteCoordinate(distgrid, xDiscrete, yDiscrete, math.sqrt(bestDistanceSquared))
        
        return distgrid
                            
                        
                
            
        
        


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
        probabilityOfOccupancy = self.grid[yDiscrete * self.mapMetaData.width + xDiscrete]
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
