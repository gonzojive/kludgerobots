import mapmodel
import particlefilter
import imageutil
import laser
import pose
import math
import sys

interactive = True

DEFAULT_CUTOFF = 0.00034

def readHw4Input():
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
    [numReadings, rightMostDegrees, leftMostDegrees, intervalDegrees] =  map(float, infile.readline().split())
    readings = map(float, infile.readlines())
    lasers = laser.Laser(readings, rightMostDegrees, intervalDegrees)
    return [initialPose, lasers]


