import math
import util
import vector

class Laser:
    def __init__(self, readings): # constructor
        self.maxRadius = 12.0  # laser only accurate to 12 feet
        self.maxTheta = math.pi  # theta in [0, pi]
        self.ranges = readings
        
    def logReadingInfo(self, ranges):
        print ranges[-1], ranges[len(ranges)/2], ranges[0]


# assuming the laser is 180 degrees, returns what angle the ith laser reading is
# given an array of laser ranges (which are arranged left to right)
def laserRangeAngle(i, numReadings):
    i = numReadings - i - 1
    if numReadings > 0:
        return (1.0 - float(i) / float(numReadings)) * pi
    else:
        return 0 #degenerate


def laserScanToVectors(laserScan):
    # filter out the maxrange stuff
    numReadings = len(laserScan)
    ranges = filter(lambda r: r[0] < 12.0, laserScan)
    result = map(lambda rng: util.polarToCartesian(rng[0], laserRangeAngle(rng[1], numReadings)), ranges)
    return result

        
    
