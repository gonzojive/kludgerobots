import vector
import random
import math
import util
import statutil
import sys
from PIL import Image

# The input coordinates are fully retarded
# Coordinates are given to us with:
#   offset = input is -0.8 in x, 3.85 in y relative to the map
#   reversal: map has y=0 at the top, input has y=0 at the bottom
#   reflection: input has -y as the up direction
#   swap: input gives us (y, x) instead of (x, y)
offset = [-0.8, 3.85]
size = [53.6, 53.0]
# mapToWorld(): hasn't been tested, is probably wrong
def mapToWorld(point):
    return [-(size[1]-point[1]+offset[1]), point[0]+offset[0]]
# worldToMap(): yes, the indices are messed up on purpose. it works
def worldToMap(point):
    return [point[1]-offset[0], size[1]+(point[0]+offset[1])]


# Helper function to grab a map index from a float value
def mapFloatIntoDiscretizedBucket(f, minFloat, denom, numBuckets):
    # f prefix float i discrete
    iBucket = int( float(f - minFloat) * denom)
    if iBucket < 0:
        return 0
    elif iBucket >= numBuckets:
        return numBuckets - 1
    else:
        return iBucket

# Even though it's the map that's offset, it will simplify calculations greatly
# if we just assume the map to be at (0,0) and translate the positions of objects
class MapModel:
    def __init__(self):
        fname = "data/gates-full-grayscale.png"
        self.im = Image.open(fname)
        print "Gates map loaded"
        [self.width, self.height] = self.im.size
        self.resolution = 0.1
        self.fWidth = self.width * self.resolution
        self.fHeight = self.height * self.resolution
        self.xMin = 0.0
        self.xMax = self.xMin + self.fWidth
        self.yMin = 0.0
        self.yMax = self.yMin + self.fHeight
        self.fSizeOfBucketDenomX = float(self.width)/float(self.xMax-self.xMin)
        self.fSizeOfBucketDenomY = float(self.height)/float(self.yMax-self.yMin)
        self.grid = list(self.im.getdata())
        self.dgrid = self.getDistanceFromObstacleGrid()


    # inBounds(): returns True if a given pose is in a legal position on the map
    # parameters:
    #   pose -- the pose to check against the map (assumed to be in map coordinates)
    def inBounds(self, pose):
        return self.probeAtPoint([pose.x, pose.y]) > 0.8

    def pointInBounds(self, xy):
        return self.probeAtPoint(xy) > 0.8


    def getDistanceFromObstacleGrid(self):
        result = None
        fname = "data/gates-distance-transform.png"
        try:
            self.distanceIm = Image.open(fname)
            result = map(lambda x: x / 10.0, list(self.distanceIm.getdata()))
            print "Distance map loaded"
        except IOError, exc:
            print "Distance map failed to load from " + fname
            sys.exit()

        # now we update the out of bounds obstacle grid locations
        # to double their distance. leftover from hw3, is this still necessary?
        for [ xDiscrete, yDiscrete ] in self.allOutOfBoundsDiscreteGridCoordinates():
            dist =  self.gridValueAtDiscreteCoordinate(result, xDiscrete, yDiscrete)
            dist = dist * 2.2
            self.setGridValueAtDiscreteCoordinate(result, xDiscrete, yDiscrete, dist)
        return result
        

    # returns the float-valued point that corresponds to the given grid cell
    def discreteGridRefToReal(self, xDiscrete, yDiscrete):
        res = self.resolution
        return [float(xDiscrete) * res + self.xMin, float(yDiscrete) * res + self.yMin]

    def gridValueAtDiscreteCoordinate(self, grid, xDiscrete, yDiscrete):
        return grid[yDiscrete * self.width + xDiscrete]

    def setGridValueAtDiscreteCoordinate(self, grid, xDiscrete, yDiscrete, value):
        grid[yDiscrete * self.width + xDiscrete] = value

    def allDiscreteGridCoordinates(self):
        for xDiscrete in xrange(0, self.width):
            for yDiscrete in xrange(0, self.height):
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
        numWildFireUpdates = min(self.height, self.width, 25)
        nearestObstacleGrid = boolgrid[:]

        numOccupied = sum([1 if x else 0 for x in boolgrid])
        numNotOccupied = sum([0 if x else 1 for x in boolgrid])

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
                for i in xrange(max(0, xDiscrete-1), min(xDiscrete+2, self.width)):
                    for j in xrange(max(0, yDiscrete-1), min(yDiscrete+2, self.height)):
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


    # given a point returns the distance to the nearest obstacle.
    # Operates in constant time
    def distanceFromObstacleAtPoint(self, pt):
        xDiscrete = mapFloatIntoDiscretizedBucket(pt[0],  self.xMin, self.fSizeOfBucketDenomX, self.width)
        yDiscrete = mapFloatIntoDiscretizedBucket(pt[1],  self.yMin, self.fSizeOfBucketDenomY, self.height)
        #print "coords: (%f, %f)  discrete: (%d, %d)" % (pt[0], pt[1], xDiscrete, yDiscrete)
        if not xDiscrete or not yDiscrete:
            return 5.0
        # Given an X, Y coordinate, the map is access via data[Y*self.width + X]
        distanceToObstacle = self.dgrid[yDiscrete * self.width + xDiscrete]
        return distanceToObstacle

            
    # returns whether the map at the given point is occupied
    def probeAtPoint(self, pt):
        xDiscrete = mapFloatIntoDiscretizedBucket(pt[0],  self.xMin, self.fSizeOfBucketDenomX, self.width)
        yDiscrete = mapFloatIntoDiscretizedBucket(pt[1],  self.yMin, self.fSizeOfBucketDenomY, self.height)
        # Given an X, Y coordinate, the map is access via data[Y*self.width + X]
        # the grid has values between 0 and 100, and -1 for unknown
        probabilityOfOccupancy = self.grid[yDiscrete * self.width + xDiscrete]
        return float(probabilityOfOccupancy) / 255.0


    def probeAtPointDiscrete(self, xDiscrete, yDiscrete):
        probabilityOfOccupancy = ord(self.grid[yDiscrete * self.width + xDiscrete])
        return float(probabilityOfOccupancy) / 255.0
