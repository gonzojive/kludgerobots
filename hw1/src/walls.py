import math
from vector import *
import ransac
from ransac import *
from vector import *
from line import *

# wall is a class that we use to to position ourself with the global compas
class Wall:
    # initialize with an infinite line that describes the line, and a set of
    # points that we used to construct this wall
    def __init__(self, line, points):
        self.line = line
        self.points = points


    def isSameWall(self, otherWall, maxAngle = d2r(10.0)):
        ang = normalizeAngle90(self.line.angleBetween(otherWall.line))
        if ang  <= maxAngle:
            return True
        else:
            return False
        
