import math
from vector import *
import util
import pylab
import hw4
import threading

def mean(l):
    return sum(l) / float(len(l))

class Circle:
    def __init__(self, radius, center):
        self.radius = radius
        self.center = center
        self.meanError = None
        self.circumferencePoints = [] # Points that we assert are on the circumference of the circle
        self.circumferenceDistances = []
        self.knownClass = "unknown"

    def calculateMetrics(self):
        """
        Returns the mean error of the points on the circumference of the circle.
        """
        self.oneNormedErrors = [abs(d - self.radius) for d in self.circumferenceDistances]
        twoNormedErrors = [e * e for e in self.oneNormedErrors]

        self.meanError = mean(twoNormedErrors)
        return self.meanError

    def angleSpannedByCircumferencePoints(self):
        """
        Returns the maximum angle spanned by any two data points
        """
        p1 = self.circumferencePoints[0]
        p2 = self.circumferencePoints[len(self.circumferencePoints) - 1]
        v1 = vector_minus(p1, self.center)
        v2 = vector_minus(p2, self.center)
        return util.normalizeAngle180(vector_angle_signed(v1, v2))

    def arffLine(self):
        """
        Outputs a line in ARFF format that we use to train an SVM.
        """
        featuresAsStrings = map(lambda y: "%s" % y, self.featureVector)
        featuresAsStrings.append("'%s'" % self.knownClass)
        return ",".join(featuresAsStrings)

    # returns a feature vector that we can use to train an SVM / classify a circle
    def get_featureVector(self):
        angleSpanned = self.angleSpannedByCircumferencePoints()
        meanSquaredError = self.meanError
        sortedErrors = self.oneNormedErrors[:]
        sortedErrors.sort()
        return [meanSquaredError,
                mean(self.oneNormedErrors),
                sortedErrors[len(sortedErrors)-1],
                sortedErrors[len(sortedErrors)-2],
                sortedErrors[len(sortedErrors)-3],
                vector_distance([1.85, 0.0000016], [angleSpanned, meanSquaredError]),
#                vector_length(self.center),
                len(self.circumferencePoints),
                angleSpanned]

    def get_error(self):
        return self.meanError

    error = property(get_error, None)
    featureVector = property(get_featureVector, None)

    def classify(self):
        """
        Returns True when this is a puck, otherwise false.
        result from logistic regression:
        mean-squared-error                    -25116.8908
        mean-1norm-error                         160.6589
        max-error                                137.7327
        2max-error                                16.6823
        3max-error                                61.1299
        dist-from-cluster                         -3.3733
        num-circumference-points                  -0.1937
        circumference-degrees                      1.6563
        Intercept                                  0.1564
        """
        u = self.featureVector
        v = [-25116.8908, 160.6589, 137.7327, 16.6823, 61.1299, -3.3733, -0.1937, 1.6563]
        hyperplaneProjection = vector_dot(u, v) + 0.1564
        return [hyperplaneProjection > -2.0, hyperplaneProjection]
        
        """
        """

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

CIRCUMFERENCE_TOLERANCE = .09
MIN_CIRCUMFERENCE_POINTS = 3

def findCircles(points, radius, cutoff=hw4.DEFAULT_CUTOFF):
    """
    Given a sequence of points relative to the robot, detects circles of a given radius and returns
    their centers.

    If trueCircles is not None, it indicates a set of circles that are known to be present in the
    data and we are in TRAINING mode.

    """
    errorDistribution = []
    for (pointIndex, point) in enumerateAndExpandPoints(points):
        # create a hypothetical circle assuming that the given scan
        # point is directly between the robot and the center of the circle
        hypotheticalCircle = Circle(radius, vector_add(point, vector_scale(vector_normalize(point), float(radius))))

        #distancesFromCenter = [vector_distance(p, hypotheticalCircle.center) for p in neighborhoodPoints(points, pointIndex)]
        #filteredDistances = filter(lambda d: d < radius + CIRCUMFERENCE_TOLERANCE, distancesFromCenter)

        # calculate how far away each other point in the scan set is
        # from the circle center/circumference
        circumferencePoints = []
        filteredDistances = []
        for p in neighborhoodPoints(points, pointIndex):
            d = vector_distance(p, hypotheticalCircle.center)
            if d < radius + CIRCUMFERENCE_TOLERANCE:
                circumferencePoints.append(p)
                filteredDistances.append(d)

        if len(filteredDistances) < MIN_CIRCUMFERENCE_POINTS:
            continue

        hypotheticalCircle.circumferencePoints = circumferencePoints
        hypotheticalCircle.circumferenceDistances = filteredDistances
        meanError = hypotheticalCircle.calculateMetrics()
            
        if len(filteredDistances) > 2:
            #if not cutoff or meanError < cutoff:
            [puckp, projection] = hypotheticalCircle.classify()
            errorDistribution.append(projection)
            if puckp:
                #print "ACCEPTED circle with mean error %f and errors %s" % (meanError, errors)
                yield hypotheticalCircle
            elif meanError < .02:
                pass #         print "Rejected circle with mean error %f and errors %s" % (meanError, errors)
    errorDistribution.sort()

    #print "Errors: %s" % errorDistribution
    pylab.hist([e for e in errorDistribution], bins=50, range=(-10, 5))
    if hw4.interactive:
        pylab.show()

        
