import mapmodel
import particlefilter
import imageutil
import laser
import pose
import sys
from hw4 import readHw4Input

class Final:
    def __init__(self):
        self.pFilter = None     # particle filter
        self.mapModel = None    # map
        self.laser = None       # laser readings
        self.pose = None        # pose from file

    # initialize()
    # Reads the test input (lr1.dat default) and puts it into self.pose and self.laser
    # Also initializes the map, which reads from the gates map and the distance map
    # Finally, initializes the particle filter with the map, start pose, and laser
    def initialize(self):
        [self.pose, self.laser] = readHw4Input()
        self.mapModel = mapmodel.MapModel()
        self.pFilter = particlefilter.ParticleFilter(self.mapModel, self.pose, self.laser)

    # run()
    # Calls the particle filter which analyzes the laser readings and returns
    # the best-fit point, the laser readings that correspond to the map, and the
    # laser readings that correspond to non-mapped objects
    # The laser readings are in cartesian coords, relative to the pose
    # Laser readings of >= 12m are not in either set
    def run(self):
        # map and object lasers are still relative to the pose at this point
        [position, mapLaserPoints, objectLaserPoints] = self.pFilter.run()
        print "Estimated position: " + position.toStr()

    # display()
    # eventually, have this code draw the position, the circles, and the other obstacles
    def display(self):
        self.mapModel.im.show()

    # userInput(): helper function for testing validity of map and point transformation
    # commented out at the bottom of the file during normal program operation
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
        
