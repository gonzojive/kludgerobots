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
        [self.pose, self.laser] = readInput()
        self.mapModel = mapmodel.MapModel()
        self.pFilter = particlefilter.ParticleFilter(self.mapModel, self.pose, self.laser)

    def run(self):
        # map and object lasers are still relative to the pose at this point
        [position, mapLaserPoints, objectLaserPoints] = self.pFilter.run()
        print "Estimated position: " + position.toStr()

    def display(self):
        self.mapModel.im.show()

    def userInput(self):
        self.mapModel.im.show()
        while 1:
            instr = raw_input("Enter a point: ")
            p = mapmodel.worldToMap(map(float, instr.split()))
            dist = self.mapModel.distanceFromObstacleAtPoint(p)
            value = self.mapModel.probeAtPoint(p)
            inB = self.mapModel.pointInBounds(p)
            print "Point (%0.2f, %0.2f): inBounds = %d, distToObstacle = %0.2f" % (p[0], p[1], inB, dist)


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
    print "Laser data loaded from " + fName
    p = map(float, infile.readline().split())
    pT = mapmodel.worldToMap(p[0:2])   # Transform to map coords
    initialPose = pose.Pose(pT[0], pT[1], p[2]*math.pi/180.0)
    infile.readline()
    readings = map(float, infile.readlines())
    lasers = laser.Laser(readings)
    return [initialPose, lasers]



if __name__ == '__main__':
    app = Final()
    app.initialize()
    #app.userInput()
    app.run()
    app.display()
