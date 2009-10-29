import rospy
import tf
import threading
from vector import *
import nav_msgs
import nav_msgs.msg
import geometry_msgs
import marshal
import pose

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
        self.fHeight = 10.0
        self.fWidth = 10.0
        self.xMax = 0.0
        self.xMin = 0.0
        self.yMin = 0.0
        self.yMin = 0.0

        self.tfBroadcaster = tf.TransformBroadcaster()
        self.currentPoseInMapFrame = Pose(initialPose.x, initialPose.y, initialPose.theta)
        #self.currentTf = [[initialPose.x, initialPose.y, 0], tf.transformations.quaternion_about_axis(initialPose.theta, [0, 0, 1]), rospy.Time.now()]
        #rospy.loginfo("Initial tf: (%0.2f, %0.2f), angle = %0.2f", initialPose[0], initialPose[1], util.r2d(initialPose[2]))
        self.tfLock = threading.Lock()  # needed in case the main thread and the filter access it at the same time

        def mapCallback(mapOccGrid):
            self.meta = mapOccGrid.info
            self.grid = mapOccGrid.data[:]
            self._annotateMapMetaData()
            rospy.loginfo("Computing obstacle distance grid: %i x %i grid", self.meta.width, self.meta.height)
            self.dgrid = self.getDistanceFromObstacleGrid()
            rospy.loginfo("Initialized Map completedly with resolution %0.2f", self.meta.resolution)
            self._initialized = True

        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, mapCallback) # listen to "map"
        self.broadcast()

    # updateMapToOdomTf(): calculate the new map->odom transformation
    # parameters:
    #   pose -- the current best guess pose (assumed to be in map coordinates)
    #   odom -- the current odometry values
    #   time -- the time at which these values were calculated
    def updateMapToOdomTf(self, guessedPoseInMapFrame, odomPoseInOdomFrame, time = None):
        self.tempTf[0][0] = guessedPoseInMapFrame.x - odomPoseInOdomFrame[0]
        self.tempTf[0][1] = guessedPoseInMapFrame.y - odom[1]
        self.tempTf[2] = time or rospy.Time.now()
        self.tempTf[1] = tf.transformations.quaternion_about_axis(guessedPoseInMapFrame.theta - odomPoseInOdomFrame[2], [0, 0, 1])
        self.tfLock.acquire()   # <--- grab the lock --->
        self.currentTf = self.tempTf[:]  # deep copy
        self.tfLock.release()   # <--- release the lock --->

    # broadcast(): send out the newest transform to tf
    def broadcast(self):
        self.tfLock.acquire()   # <--- grab the lock --->
        # FIXME FIXME FIXME URGENT THIS IS WRONGGGGGGGGGGGGG
        self.tfBroadcaster.sendTransform(self.currentPoseInMapFrame.x,
                                         self.currentPoseInMapFrame.y,
                                         tf.transformations.quaternion_about_axis(initialPose.theta, [0.0, 0.0, 1.0]),
                                         # publish the transform from the odom frame to the map frame.
                                         "odom", "map")
        self.tfLock.release()   # <--- release the lock --->
        #rospy.loginfo("Sent broadcast")

    # inBounds(): returns True if a given pose is in a legal position on the map
    # parameters:
    #   pose -- the pose to check against the map (assumed to be in map coordinates)
    def inBounds(self, pose):
        return self.probeAtPoint([pose.x, pose.y]) < 0.5

    def _annotateMapMetaData(self):
        meta = self.meta

        # FIXME: account for the orientation of the map
        resolution = meta.resolution
        self.fHeight = resolution * float(meta.height)
        self.fWidth = resolution * float(meta.height)
        
        self.xMin = meta.origin.position.x
        self.xMax = self.xMin + self.fWidth
        
        self.yMin = meta.origin.position.y
        self.yMax= self.yMin + self.fHeight

    def getDistanceFromObstacleGrid(self):
        result = None
        fname = '/tmp/myGatesMapDist.marshal'
        try:
            stream = file(fname, 'r')
            result = marshal.load(stream)
            return result
        except IOError, exc:
            result = self.computeDistanceFromObstacleGrid()
            stream = file(fname, 'w')
            marshal.dump(result, stream)
            return result
            
    def computeDistanceFromObstacleGrid(self):
        # create a row-major grid that holds the nearest obstacle from
        # each point in the original map grid

        def discreteGridRefToReal(xDiscrete, yDiscrete):
            res = self.meta.resolution
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

        boolgrid = map(obstaclepFromProbability, map(ord, self.grid))


        # given discrete grid indexes returns these functions 
        def gridValueAtDiscreteCoordinate(grid, xDiscrete, yDiscrete):
            return grid[yDiscrete * self.meta.width + xDiscrete]

        def setGridValueAtDiscreteCoordinate(grid, xDiscrete, yDiscrete, value):
            grid[yDiscrete * self.meta.width + xDiscrete] = value

        numWildFireUpdates = min(self.meta.height, self.meta.width)

        def allDiscreteGridCoordinates():
            for xDiscrete in xrange(0, self.meta.width):
                for yDiscrete in xrange(0, self.meta.height):
                    yield [xDiscrete, yDiscrete]

        nearestObstacleGrid = boolgrid[:]
        
        numOccupied = sum([1 if x else 0 for x in boolgrid])
        numNotOccupied = sum([0 if x else 1 for x in boolgrid])
        rospy.loginfo("%i empty %i non-empty grid cells e.g. %s", numNotOccupied, numOccupied, ord(self.grid[0]))

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
                for i in xrange(max(0, xDiscrete-1), min(xDiscrete+2, self.meta.width)):
                    for j in xrange(max(0, yDiscrete-1), min(yDiscrete+2, self.meta.height)):
                        yield [i, j]

            return [gridValueAtDiscreteCoordinate(nearestObstacleGrid, x, y) for [x, y] in pts()]
        
        for i in xrange(0, numWildFireUpdates):
            numUpdatedGridCells = 0
            for [xDiscrete, yDiscrete] in allDiscreteGridCoordinates():
                # point in space does this grid coordinate corresponds to
                realPoint = discreteGridRefToReal(xDiscrete, yDiscrete)
                def distSquaredToObstacle(pt):
                    vToPt = vector_minus(pt, realPoint)
                    return vector_length_squared(vToPt)
                        
                # keep track of the closest obstacle
                nearestObstacle = gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
                nearestObstacleDistSquared = -1
                if nearestObstacle:
                    nearestObstacleDistSquared = distSquaredToObstacle(nearestObstacle)
                    
                # get a list of obstacles that are near neighboring cells
                nearbyObstacles = [x for x in knownNearbyObstacles(xDiscrete, yDiscrete)]

                for nearbyObstacle in nearbyObstacles:
                    if nearbyObstacle:
                        # if this nearby obstacle is closer than others, set it as the best
                        nearbyObstacleDistSquared = distSquaredToObstacle(nearbyObstacle)
                        if not nearestObstacle or nearbyObstacleDistSquared + .001 < nearestObstacleDistSquared:
                            nearestObstacleDistSquared = nearbyObstacleDistSquared
                            nearestObstacle = nearbyObstacle
                            numUpdatedGridCells += 1
                setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)
            rospy.loginfo("%ith wildfire updated %i grid cells", i, numUpdatedGridCells)
            if numUpdatedGridCells == 0:
                break

        distgrid = nearestObstacleGrid[:]
        # given the grid that maps points to nearest obstacles,
        # compute the actual distance from each cell to its nearest
        # obstacle and return a grid with the result
        for [xDiscrete, yDiscrete] in allDiscreteGridCoordinates():
            # get the point corresponding to this cell
            realPoint = discreteGridRefToReal(xDiscrete, yDiscrete)
            # get the nearest obstacle to that point
            nearestObstacle = gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
            # find the distance between them
            vToNearestObstacle = vector_minus(nearestObstacle, realPoint)
            nearestObstacleDist = vector_length(vToNearestObstacle)
            # set the distgrid value
            setGridValueAtDiscreteCoordinate(distgrid, xDiscrete, yDiscrete, nearestObstacleDist)
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


    # givena  point returns the distance to the nearest obstacle.  Operates in constant time
    def distanceFromObstacleAtPoint(self, pt):
        if self.initializedp():
            xDiscrete = mapFloatIntoDiscretizedBucket(pt[0],  self.xMin, self.xMax, self.meta.width)
            yDiscrete = mapFloatIntoDiscretizedBucket(pt[1],  self.yMin, self.yMax, self.meta.height)
            # Given an X, Y coordinate, the map is access via data[Y*meta.width + X]
            # the grid has values between 0 and 100, and -1 for unknown
            distanceToObstacle = self.dgrid[yDiscrete * self.meta.width + xDiscrete]
            return distanceToObstacle
        else:
            return -1
            
    # returns whether the map at the given point is occupied
    def probeAtPoint(self, pt):
        # here we the
        meta = self.meta
        xDiscrete = mapFloatIntoDiscretizedBucket(pt[0],  self.xMin, self.xMax, meta.width)
        yDiscrete = mapFloatIntoDiscretizedBucket(pt[1],  self.yMin, self.yMax, meta.height)
        # Given an X, Y coordinate, the map is access via data[Y*meta.width + X]
        # the grid has values between 0 and 100, and -1 for unknown
        probabilityOfOccupancy = ord(self.grid[yDiscrete * meta.width + xDiscrete])
        #rospy.loginfo("xD = %d, yD = %d", xDiscrete, yDiscrete)       
        #rospy.loginfo("val = %d", ord(probabilityOfOccupancy))
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
