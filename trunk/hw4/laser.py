import math
import util
import vector

# Laser class
# members: polarPoints, a list of [r, theta] values giving the laser readings
#          points, a list of [x, y] values giving the laser readings

class Laser:
    def __init__(self, readings, rightMostDegrees, intervalDegrees): # constructor
        self.maxRadius = 12.0  # laser only accurate to 12 feet
        self.maxTheta = math.pi  # theta in [0, pi]
        polarPoints = [(r, util.d2r(rightMostDegrees + intervalDegrees*float(i))) for (i, r) in enumerate(readings)]
        self.polarPoints = polarPoints
        self.points = [util.polarToCartesian(r, theta) for (r, theta) in polarPoints]
        
    def logReadingInfo(self, ranges):
        print ranges[-1], ranges[len(ranges)/2], ranges[0]

