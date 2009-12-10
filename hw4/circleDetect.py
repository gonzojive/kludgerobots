import math
from vector import *
import pylab
import hw4
import threading

class Circle:
    def __init__(self, radius, center):
        self.radius = radius
        self.center = center
        self.error = None

    def __repr__(self):
        return "<Circle radius %f at %s >" % (self.radius, self.center)

def neighborhoodPoints(points, pointIndex, n=40): # take 40 degrees worth of points
    numEitherSide = int((n - 1) / 2)
    #return points[max(0, pointIndex - numEitherSide):pointIndex] + points[pointIndex+1:min(len(points), pointIndex + numEitherSide)]
    return points[max(0, pointIndex - numEitherSide):min(len(points), pointIndex + numEitherSide)]

# adds linearly interpolated points in between laser readings
def enumerateAndExpandPoints(points):
    for (pointIndex, point) in enumerate(points):
        yield (pointIndex, point)
        nextPoint = points[pointIndex+1] if pointIndex < len(points) - 1 else None
        if nextPoint:
            yield (pointIndex, vector_scale(vector_add(nextPoint, point), .5))


def findCircles(points, radius, cutoff=hw4.DEFAULT_CUTOFF):
    """
    Given a sequence of points relative to the robot, detects circles of a given radius and returns
    their centers.
    """
    errorDistribution = []
    for (pointIndex, point) in enumerateAndExpandPoints(points):
        # create a hypothetical circle assuming that the given scan
        # point is directly between the robot and the center of the circle
        hypotheticalCircle = Circle(radius, vector_add(point, vector_scale(vector_normalize(point), float(radius))))
        # calculate how far away each other point in the scan set is
        # from the circle
        distancesFromCenter = [vector_distance(p, hypotheticalCircle.center) for p in neighborhoodPoints(points, pointIndex)]
        filteredDistances = filter(lambda d: d < radius + .09, distancesFromCenter)

        if len(filteredDistances) == 0:
            continue

        errors = [abs(d - radius) for d in filteredDistances]
        errors = [e * e for e in errors]
        meanError = sum(errors) / float(len(filteredDistances))
            
        if len(filteredDistances) > 2:
            errorDistribution.append(meanError)
            hypotheticalCircle.error = meanError
            if meanError < cutoff:
                #print "ACCEPTED circle with mean error %f and errors %s" % (meanError, errors)
                yield hypotheticalCircle
            elif meanError < .02:
                pass #         print "Rejected circle with mean error %f and errors %s" % (meanError, errors)
    errorDistribution.sort()

    print "Errors: %s" % errorDistribution
    pylab.hist([e for e in errorDistribution], bins=50, range=(0, .002))
    if hw4.interactive:
        pylab.show()

        
