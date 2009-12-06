import mapmodel
import particlefilter
import imageutil
import laser
import pose
import sys
import math

class Final:
    def __init__(self):
        self.pFilter = None
        self.mapModel = None
        self.laser = None
        self.pose = None

    def initialize(self):
        self.mapModel = mapmodel.MapModel()
        [self.pose, self.laser] = readInput()
        self.pFilter = particlefilter.ParticleFilter(self.mapModel, self.pose, self.laser)

    def display(self):
        pass


def readInput():
    if len(sys.argv) > 1:
        fName = "data/lr" + sys.argv[1] + ".dat"
    else:
        fName = "data/lr1.dat"
    try:
        infile = open(fName)
    except:
        print fName + " does not exist"
        sys.exit()
    p = map(float, infile.readline().split())
    initialPose = pose.Pose(p[0], p[1], p[2]*math.pi/180.0)
    infile.readline()
    readings = map(float, infile.readlines())
    lasers = laser.Laser(readings)
    return [initialPose, lasers]


if __name__ == '__main__':
    app = Final()
    app.initialize()
    app.display()
