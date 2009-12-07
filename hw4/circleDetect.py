import math
from vector import *

class Circle:
    def __init__(self, radius, center):
        self.radius = radius
        self.center = center

    def __repr__(self):
        return "<Circle radius %f at %s >" % (self.radius, self.center)

#def nearbyPoints(points, pointIndex, n=5):
#    i = 
    

def findCircles(points, radius):
    """
    Given a sequence of points relative to the robot, detects circles of a given radius and returns
    their centers.
    """
    for (pointIndex, point) in enumerate(points):
        # create a hypothetical circle assuming that the given scan
        # point is directly between the robot and the center of the circle
        hypotheticalCircle = Circle(radius, vector_add(point, vector_scale(vector_normalize(point), float(radius))))
        # calculate how far away each other point in the scan set is
        # from the circle
        distancesFromCenter = [vector_distance(p, hypotheticalCircle.center) for p in points]
        filteredDistances = filter(lambda d: d < radius + .1, distancesFromCenter)
        if len(filteredDistances) > 2:
            errors = [abs(d - radius) for d in filteredDistances]
            meanError = sum(filteredDistances) / float(len(filteredDistances))
            if meanError < .2:
                print "ACCEPTED circle with mean error %f and errors %s" % (meanError, errors)
                yield hypotheticalCircle
            elif meanError < .25:
                print "Rejected circle with mean error %f and errors %s" % (meanError, errors)
        
