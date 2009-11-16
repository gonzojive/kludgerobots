#python
import threading
import marshal
import random
import math

#ros
import roslib; roslib.load_manifest('hw2')
import rospy
#import tf
import nav_msgs
import nav_msgs.msg
import geometry_msgs
from heapq import *

# kludge
import util
import quaternion
import vector
from vector import *
from viz import Visualizer

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

# VOCABULARY:
#  dref = discrete reference (point).  an [integer, integer] index into the array
class DiscretizedGrid:
    def __init__(self, origin, size, resolution):
        self.origin = origin
        self.size = size
        self.resolution = resolution
        self.data = None

    def copy(self):
        d = [x for x in self.data] if self.data else None
        return DiscretizedGrid(self.origin, self.size, self.resolution, d)

    def valueAtDRef(self, dPoint):
        return self.grid[dPoint[1] * self.size[0] + dPoint[0]]

    def setValueAtDRef(self, grid, dPoint, value):
        grid[dPoint[1] * self.size[0] + dPoint[0]] = value

    def allDRefs(self):
        for xDiscrete in xrange(0, self.size[0]):
            for yDiscrete in xrange(0, self.size[1]):
                yield [xDiscrete, yDiscrete]

    # transforming from discrete to real coordinates.  Note that right
    # now we ignore the orientation of the grid
    def dRefToReal(self, dPoint):
        res = self.resolution
        return map(lambda d, o : (float(d) * res + o), dPoint, self.origin)


def obstaclepGrid(occGrid):
    # set up the bounds
    res = occGrid.info.resolution
    offset = [occGrid.info.origin.position.x, occGrid.info.origin.position.y]
    offset = vector_add(offset, [res / 2.0, res / 2.0])
    dim = vector_add([0, 0], [occGrid.info.width, occGrid.info.height])
    g = DiscretizedGrid(offset, dim, res)
    g.grid = [True if ord(d) == 100 else False for d in occGrid.data]
    return g
    
def computeAndVizGradientGrid(occGrid, v):
    v.vizPoints([ [0,0,.1],[-1,0,.1], [1,0,0], [50, 0, 0]])
    v.vizArrow([0,0,0], .4)
    v.vizArrow([0,0,0], 2.4)
    v.vizArrow([0,0,0], 4.9)
    obsGrid = obstaclepGrid(occGrid)
    def goodDRef(dref):
        return obsGrid.valueAtDRef(dref)
    vpoints = filter(lambda x: x,
                     [obsGrid.dRefToReal(pt) if goodDRef(pt) else False for pt in obsGrid.allDRefs()])
    rospy.loginfo("Visualizing %i points" % len(vpoints))
    #v.vizPoints(vpoints)
    v.otherStuff()
    v.vizArrows([ ( [3, 0], [5, 6]), ( [4, 0], [5, 6]) ] )
    

def fastMarch(inGrid, updateFn):
    class FMCell:
        def __init__(self, obstaclep):
            self.value = 0 if obstaclep else None

    
            
    grid = inGrid.copy()
    grid.data = [FMCell(obstaclep) for obstaclep in grid.data]
    aliveSet = set(grid.data)
    farSet = set(aliveSet)

