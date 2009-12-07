from hw4 import readHw4Input
from circleDetect import findCircles

def main():
    [pose, laser] = readHw4Input()
    for radius in [.3]:
        circleCenters = [c for c in findCircles(laser.points, radius)]
        print "Found %i circle centers with radius %f" % (len(circleCenters), radius)
        print circleCenters


if __name__ == '__main__':
    main()
