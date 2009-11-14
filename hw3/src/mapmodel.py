import rospy
import tf
import threading
import vector
from vector import *
import nav_msgs
import nav_msgs.msg
import geometry_msgs
import marshal
import pose
import quaternion
import random
import math
import util
import statutil
import sys

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

        # last time the robot position within the map was updated
        self.translationMapToOdom = None
        self.rotationMapToOdom = None
        self.lastUpdateTime = rospy.Time.now()
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.currentPoseInMapFrame = pose.Pose(initialPose.x, initialPose.y, initialPose.theta)
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
        # translate from the
        self.tfLock.acquire()   # <--- grab the lock --->
        #1. Rotate the odom frame
        guessedTheta = guessedPoseInMapFrame.theta
        odomTheta = odomPoseInOdomFrame[2]
        rotationTheta = odomTheta - guessedTheta
        quat = tf.transformations.quaternion_about_axis( rotationTheta, [0, 0, 1])

        #2. figure out the translation from the map to the odom frame
        # apply the rotation to the point in the map frame
        mapCoordRotated = quaternion.rotateVectorWithQuaternion([guessedPoseInMapFrame.x , guessedPoseInMapFrame.y, 0.0],
                                                               quat)

        # odom - mapcoordRotated
        translation = vector.vector_minus([guessedPoseInMapFrame.x , guessedPoseInMapFrame.y, 0.0], #mapCoordRotated,
                                          [ odomPoseInOdomFrame[0], odomPoseInOdomFrame[1], 0.0],
                                          )

        rospy.loginfo("Determined map => odom transform.  Translation: (%0.2f, %0.2f), angle = %0.2f degrees",
                      translation[0], translation[1], util.r2d(rotationTheta))

        self.translationMapToOdom = translation
        self.rotationMapToOdom = quat
        self.lastUpdateTime = time or rospy.Time.now()
        self.tfLock.release()   # <--- release the lock --->

    # broadcast(): send out the newest transform to tf
    def broadcast(self):
        self.tfLock.acquire()   # <--- grab the lock --->
        if self.translationMapToOdom:
            rospy.loginfo("broadcasting map => odom transform.  translation: (%0.2f, %0.2f)",
                          self.translationMapToOdom[0], self.translationMapToOdom[1])

            # FIXME FIXME FIXME URGENT THIS IS WRONGGGGGGGGGGGGG
            # sendTransform(translation, rotation, time, child, parent)
            # where does the ODOM think we are?
            # where do we actually think we are?
            # publish the transform from the odom frame to the map frame.
            self.tfBroadcaster.sendTransform(self.translationMapToOdom,
                                             self.rotationMapToOdom,
                                             self.lastUpdateTime,
                                             "odom", "map")
        self.tfLock.release()   # <--- release the lock --->
        #rospy.loginfo("Sent broadcast")

    # inBounds(): returns True if a given pose is in a legal position on the map
    # parameters:
    #   pose -- the pose to check against the map (assumed to be in map coordinates)
    def inBounds(self, pose):
        return self.probeAtPoint([pose.x, pose.y]) < 0.5

    def pointInBounds(self, xy):
        return self.probeAtPoint(xy) < 0.5

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
        fname = sys.path[0] + '/../' + 'myGatesMapDist.marshal'
        try:
            stream = file(fname, 'r')
            result = marshal.load(stream)
            rospy.loginfo("Map was successfully loaded from " + fname);
        except IOError, exc:
            rospy.loginfo("Map failed to load from " + fname);
            result = self.computeDistanceFromObstacleGrid()
            stream = file(fname, 'w')
            marshal.dump(result, stream)

        # now we update the out of bounds obstacle grid locations to double their distance
        for [ xDiscrete, yDiscrete ] in self.allOutOfBoundsDiscreteGridCoordinates():
            dist =  self.gridValueAtDiscreteCoordinate(result, xDiscrete, yDiscrete)
            dist = dist * 2.2
            self.setGridValueAtDiscreteCoordinate(result, xDiscrete, yDiscrete, dist)

        return result
        

    # returns the float-valued point that corresponds to the given grid cell
    def discreteGridRefToReal(self, xDiscrete, yDiscrete):
        res = self.meta.resolution
        return [float(xDiscrete) * res + self.xMin, float(yDiscrete) * res + self.yMin]

    def gridValueAtDiscreteCoordinate(self, grid, xDiscrete, yDiscrete):
        return grid[yDiscrete * self.meta.width + xDiscrete]

    def setGridValueAtDiscreteCoordinate(self, grid, xDiscrete, yDiscrete, value):
        grid[yDiscrete * self.meta.width + xDiscrete] = value

    def allDiscreteGridCoordinates(self):
        for xDiscrete in xrange(0, self.meta.width):
            for yDiscrete in xrange(0, self.meta.height):
                yield [xDiscrete, yDiscrete]

    # generates all in-bound grid coords
    def allInBoundsDiscreteGridCoordinates(self):
        for pt in self.allDiscreteGridCoordinates():
            if self.pointInBounds(pt):
                yield pt

    def allOutOfBoundsDiscreteGridCoordinates(self):
        for pt in self.allDiscreteGridCoordinates():
            if not self.pointInBounds(pt):
                yield pt

    def generatePosesOverWholeMap(self, n):
        i = 0
        while True:
            if i > n:
                break
            x = int(self.meta.width * random.random())
            y = int(self.meta.height * random.random())
            realPt = self.discreteGridRefToReal(x, y)
            if self.pointInBounds(realPt):
                i += 1
                # rando theta
                theta = random.random() * 2.0 * math.pi
                yield pose.Pose(realPt[0], realPt[1], theta)


    def generatePosesNearPose(self, p, stdDeviation, n):
        for i in xrange(0, n):
            realPt = statutil.randomMultivariateGaussian([ [p.x, stdDeviation],
                                                           [p.y, stdDeviation]])
            if self.pointInBounds(realPt):
                # rando theta
                theta = random.random() * 2.0 * math.pi
                yield pose.Pose(realPt[0], realPt[1], theta)

    def computeDistanceFromObstacleGrid(self):
        # create a row-major grid that holds the nearest obstacle from
        # each point in the original map grid

        # the grid has a value between 0 and 100 that indicates the
        # probability that an element of the grid is occupied.  We
        # assume anything with a value >= 10 is occupied.  If the
        # value is -1 it means it is unknown and we assume that it is
        # an obstacle
        probs = {}
        def obstaclepFromProbability(p):
            probs[p] = True
            if p > 0 and p != 255:
                return True
            else:
                return False


        boolgrid = map(obstaclepFromProbability, map(ord, self.grid))

        rospy.loginfo("probs: %s", probs)
        # given discrete grid indexes returns these functions 


        numWildFireUpdates = min(self.meta.height, self.meta.width, 25)

        nearestObstacleGrid = boolgrid[:]
        
        numOccupied = sum([1 if x else 0 for x in boolgrid])
        numNotOccupied = sum([0 if x else 1 for x in boolgrid])
        rospy.loginfo("%i empty %i non-empty grid cells e.g. %s", numNotOccupied, numOccupied, ord(self.grid[0]))

        # Initially set the nearest obstacle at each point to itself
        # if there is an obstacle there, or none if there is not an
        # obstacle there
        for [xDiscrete, yDiscrete] in self.allDiscreteGridCoordinates():
            nearestObstacle = None
            if self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete):
                nearestObstacle = self.discreteGridRefToReal(xDiscrete, yDiscrete)
            self.setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)


        
        # returns a list of obstacles that surround the given grid coordinate
        def knownNearbyObstacles(xDiscrete, yDiscrete):
            def pts ():
                for i in xrange(max(0, xDiscrete-1), min(xDiscrete+2, self.meta.width)):
                    for j in xrange(max(0, yDiscrete-1), min(yDiscrete+2, self.meta.height)):
                        yield [i, j]

            return [self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, x, y) for [x, y] in pts()]
        
        for i in xrange(0, numWildFireUpdates):
            numUpdatedGridCells = 0
            for [xDiscrete, yDiscrete] in self.allDiscreteGridCoordinates():
                # point in space does this grid coordinate corresponds to
                realPoint = self.discreteGridRefToReal(xDiscrete, yDiscrete)
                def distSquaredToObstacle(pt):
                    vToPt = vector_minus(pt, realPoint)
                    return vector_length_squared(vToPt)
                        
                # keep track of the closest obstacle
                nearestObstacle = self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
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
                self.setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)
            rospy.loginfo("%ith wildfire updated %i grid cells", i, numUpdatedGridCells)
            if numUpdatedGridCells == 0:
                break

        distgrid = nearestObstacleGrid[:]
        # given the grid that maps points to nearest obstacles,
        # compute the actual distance from each cell to its nearest
        # obstacle and return a grid with the result
        for [xDiscrete, yDiscrete] in self.allDiscreteGridCoordinates():
            # get the point corresponding to this cell
            realPoint = self.discreteGridRefToReal(xDiscrete, yDiscrete)
            # get the nearest obstacle to that point
            nearestObstacle = self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
            nearestObstacleDist = 2.5 # default to 2.5 meters
            if nearestObstacle:
                # find the distance between them
                vToNearestObstacle = vector_minus(nearestObstacle, realPoint)
                nearestObstacleDist = vector_length(vToNearestObstacle)
                # set the distgrid value
            self.setGridValueAtDiscreteCoordinate(distgrid, xDiscrete, yDiscrete, nearestObstacleDist)
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
            if not xDiscrete or not yDiscrete:
                return 5.0
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
    
    def computeDistanceFromObstacleGridWild(self):
        # create a row-major grid that holds the nearest obstacle from

        # the grid has a value between 0 and 100 that indicates the
        # probability that an element of the grid is occupied.  We
        # assume anything with a value >= 10 is occupied.  If the
        # value is -1 it means it is unknown and we assume that it is
        # an obstacle
        probs = {}
        def obstaclepFromProbability(p):
            probs[p] = True
            if p > 0 and p != 255:
                return True
            else:
                return False


        boolgrid = map(obstaclepFromProbability, map(ord, self.grid))

        rospy.loginfo("probs: %s", probs)

        # given discrete grid indexes returns these functions 
        numWildFireUpdates = min(self.meta.height, self.meta.width, 25)

        unburnedList = [x for x in self.allDiscreteGridCoordinates()]
        # stores whether each element of the grid has been burned
        nearestObstacleGrid = boolgrid[:]

        # Initially set the nearest obstacle at each point to itself
        # if there is an obstacle there, or none if there is not an
        # obstacle there
        for [xDiscrete, yDiscrete] in self.allDiscreteGridCoordinates():
            nearestObstacle = None
            if self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete):
                nearestObstacle = self.discreteGridRefToReal(xDiscrete, yDiscrete)
            self.setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)


        
        # returns a list of obstacles that surround the given grid coordinate
        def knownNearbyObstacles(xDiscrete, yDiscrete):
            def pts ():
                for i in xrange(max(0, xDiscrete-1), min(xDiscrete+2, self.meta.width)):
                    for j in xrange(max(0, yDiscrete-1), min(yDiscrete+2, self.meta.height)):
                        yield [i, j]

            return [self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, x, y) for [x, y] in pts()]
        
        for i in xrange(0, numWildFireUpdates):
            numUpdatedGridCells = 0
            for [xDiscrete, yDiscrete] in self.allDiscreteGridCoordinates():
                # point in space does this grid coordinate corresponds to
                realPoint = self.discreteGridRefToReal(xDiscrete, yDiscrete)
                def distSquaredToObstacle(pt):
                    vToPt = vector_minus(pt, realPoint)
                    return vector_length_squared(vToPt)
                        
                # keep track of the closest obstacle
                nearestObstacle = self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
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
                self.setGridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete, nearestObstacle)
            rospy.loginfo("%ith wildfire updated %i grid cells", i, numUpdatedGridCells)
            if numUpdatedGridCells == 0:
                break

        distgrid = nearestObstacleGrid[:]
        # given the grid that maps points to nearest obstacles,
        # compute the actual distance from each cell to its nearest
        # obstacle and return a grid with the result
        for [xDiscrete, yDiscrete] in self.allDiscreteGridCoordinates():
            # get the point corresponding to this cell
            realPoint = self.discreteGridRefToReal(xDiscrete, yDiscrete)
            # get the nearest obstacle to that point
            nearestObstacle = self.gridValueAtDiscreteCoordinate(nearestObstacleGrid, xDiscrete, yDiscrete)
            nearestObstacleDist = 2.5 # default to 2.5 meters
            if nearestObstacle:
                # find the distance between them
                vToNearestObstacle = vector_minus(nearestObstacle, realPoint)
                nearestObstacleDist = vector_length(vToNearestObstacle)
                # set the distgrid value
            self.setGridValueAtDiscreteCoordinate(distgrid, xDiscrete, yDiscrete, nearestObstacleDist)
        return distgrid


