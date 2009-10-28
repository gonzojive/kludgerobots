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
    def __init__(self, initialPose):
        self._initialized = False

        # parameters used to discretize floating point map coords into buckets
        self.fHeight = 10.0
        self.fWidth = 10.0
        self.xMax = 0.0
        self.xMin = 0.0
        self.yMin = 0.0
        self.yMin = 0.0

        self.tfBroadcaster = tf.TransformBroadcaster()
        self.currentTf = [[initialPose[0], initialPose[1], 0], tf.transformations.quaternion_about_axis(initialPose[2], [0, 0, 1]), rospy.Time.now()]
        #rospy.loginfo("Initial tf: (%0.2f, %0.2f), angle = %0.2f", initialPose[0], initialPose[1], util.r2d(initialPose[2]))
        self.tempTf = self.currentTf[:]
        self.tfLock = threading.Lock()  # needed in case the main thread and the filter access it at the same time

        def mapCallback(mapOccGrid):
            self.mapMetaData = mapOccGrid.info
            self.grid = mapOccGrid.data
            self._annotateMapMetaData()
            rospy.loginfo("Computing obstacle distance grid: %i x %i grid", self.mapMetaData.width, self.mapMetaData.height)
            
            #self.dgrid = self.computeDistanceFromObstacleGrid()
            rospy.loginfo("Initialized Map completedly")
            self._initialized = True

        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, mapCallback) # listen to "map"
        self.broadcast()


    # updateMapToOdomTf(): calculate the new map->odom transformation
    # parameters:
    #   pose -- the current best guess pose (assumed to be in map coordinates)
    #   odom -- the current odometry values
    #   time -- the time at which these values were calculated
    def updateMapToOdomTf(self, pose, odom, time = None):
        self.tempTf[0][0] = pose.x - odom[0]
        self.tempTf[0][1] = pose.y - odom[1]
        self.tempTf[2] = time or rospy.Time.now()
        self.tempTf[1] = tf.transformations.quaternion_about_axis(pose.theta-odom[2], [0, 0, 1])
        self.tfLock.acquire()   # <--- grab the lock --->
        self.currentTf = self.tempTf[:]  # deep copy
        self.tfLock.release()   # <--- release the lock --->

    # broadcast(): send out the newest transform to tf
    def broadcast(self):
        self.tfLock.acquire()   # <--- grab the lock --->
        self.tfBroadcaster.sendTransform(self.currentTf[0], self.currentTf[1], self.currentTf[2], "odom", "map")
        self.tfLock.release()   # <--- release the lock --->
        #rospy.loginfo("Sent broadcast")

    # inBounds(): returns True if a given pose is in a legal position on the map
    # parameters:
    #   pose -- the pose to check against the map (assumed to be in map coordinates)
    def inBounds(self, pose):
        return self.probeAtPoint([pose.x, pose.y]) < 0.5


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

        # given discrete grid indexes returns these functions 
        def gridValueAtDiscreteCoordinate(grid, xDiscrete, yDiscrete):
            return grid[yDiscrete * self.mapMetaData.width + xDiscrete]

        def setGridValueAtDiscreteCoordinate(grid, xDiscrete, yDiscrete, value):
            grid[yDiscrete * self.mapMetaData.width + xDiscrete] = value

        numWildFireUpdates = min(self.mapMetaData.height, self.mapMetaData.width)

        def allDiscreteGridCoordinates():
            for xDiscrete in xrange(0, self.mapMetaData.width):
                for yDiscrete in xrange(0, self.mapMetaData.height):
                    yield [xDiscrete, yDiscrete]

        nearestObstacleGrid = boolgrid[:]

        # Initially set the nearest obstacle at each point to itself
        # if there is an obstacle there, or none if there is not an
        # obstacle there
        for [xDiscrete, yDiscrete] in allDiscreteGridCoordinates():
            nearestObstacle = None
            if gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete):
                nearestObstacle = discreteGridRefToReal(xDiscrete, yDiscrete)
            setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)

        # returns a list of obstacles that surround the given grid coordinate
        def knownNearbyObstacles(xDiscrete, yDiscrete):
            def pts ():
                for i in xrange(max(0, xDiscrete), min(xDiscrete+1, self.meta.width - 1)):
                    for j in xrange(max(0, yDiscrete), min(yDiscrete+1, self.meta.height - 1)):
                        yield [i, j]

            return [gridValueAtDiscreteCoordinate(nearestObstacleGrid, x, y) for [x, y] in pts()]
        
        for i in xrange(0, numWildFireUpdates):
            for [xDiscrete, yDiscrete] in allDiscreteGridCoordinates():
                # point in space does this grid coordinate corresponds to
                realPoint = discreteGridRefToReal(xDiscrete, yDiscrete)
                # keep track of the closest obstacle
                nearestObstacle = None
                nearestObstacleDistSquared = -1
                # get a list of obstacles that are near neighboring cells
                nearbyObstacles = knownNearbyObstacles(xDiscrete, yDiscrete)

                for nearbyObstacle in nearbyObstacles:
                    if nearbyObstacle:
                        vToNearbyObstcle = vector_minus(nearbyObstacle, realPoint)
                        nearbyObstacleDistSquared = vector_length_squared(vToNearbyObstcle)
                        if not nearestObstacle or nearbyObstacleDistSquared < nearestObstacleDistSquared:
                            nearestObstacleDistSquared = nearbyObstacleDistSquared
                            nearestObstacle = nearbyObstacle
                setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)

        distgrid = nearestObstacleGrid[:]
        # given the grid that maps points to nearest obstacles,
        # compute the actual distance from each cell to its nearest
        # obstacle and return a grid with the result
        for [xDiscrete, yDiscrete] in allDiscreteGridCoordinates():
            realPoint = discreteGridRefToReal(xDiscrete, yDiscrete)
            nearestObstacle = gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
            vToNearbyObstcle = vector_minus(nearbyObstacle, realPoint)
            nearbyObstacleDist = vector_length(vToNearestObstcle)
            setGridValueAtDiscreteCoordinate(distgrid, xDiscrete, yDiscrete, nearbyObstacleDist)
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
        probabilityOfOccupancy = self.grid[yDiscrete * meta.width + xDiscrete]
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
