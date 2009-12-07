import mapmodel
import particlefilter
import imageutil
import laser
import pose
import sys
from hw4 import readHw4Input

class Final:
    def __init__(self):
        self.pFilter = None
        self.mapModel = None
        self.laser = None
        self.pose = None

    def initialize(self):
        [self.pose, self.laser] = readHw4Input()
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


if __name__ == '__main__':
    if len(sys.argv) == 1:
        app = Final()
        app.initialize()
        #app.userInput()
        app.run()
        app.display()
        
