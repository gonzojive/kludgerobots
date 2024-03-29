import math
import random
import rospy

# returns the probability of the value given the mean and standard
# deviation of a gaussian distribution
def gaussianProbability(mean, standardDeviation, value):
    variance = standardDeviation * standardDeviation
    coef = 1.0 / math.sqrt(2.0 * math.pi * variance)
    diffFromMean = float(value) - float(mean)
    innerExp = (-1.0 * diffFromMean * diffFromMean) / (2.0 * variance)
    return coef * math.exp(innerExp)

def mean(values):
    if len(values) > 0:
        num = float(len(values))
        return sum(values) / num
    else:
        return 0

# fundamental theory of probability:
# variance([randomGaussian(0.0, 1.0) for n in xrange(0, 200)]) => 1.1129958700263398
# variance([randomGaussian(0.0, 1.0) for n in xrange(0, 200000)]) => 0.99666458910502709
def variance(values):
    avg = mean(values)
    sumOfSquareDistancesToMean = 0.0
    for v in values:
        delta = avg - v
        deltaSqr = delta * delta
        sumOfSquareDistancesToMean += deltaSqr
    return sumOfSquareDistancesToMean / float(len(values))

# returns a random number distributed according to the normal distribution with
# the provided mean and sigma
def randomGaussian(mean, standardDeviation):
    # From Knuth v2, 3rd ed, p122
    w = 0
    y1 = 0
    y2 = 0
    while True:
        x1 = 2.0 * random.random() - 1.0;
        x2 = 2.0 * random.random() - 1.0;
        w = x1 * x1 + x2 * x2; # square it
        if w < 1.0 and w > 0.0:
            break
    return mean + standardDeviation * x1 * math.sqrt (-2.0 * math.log (w) / w);

# given an array of N pairs (where first is mean and second is sigma), returns
# a list of length N with N random gaussians
def randomMultivariateGaussian(arrayOfMeansAndSigmas):
    return [ randomGaussian(u, sigma) for [u, sigma] in arrayOfMeansAndSigmas];


def randomUniform(min, max):
    return (max - min) * random.random() + min

def randomMultivariateUniform(arrayOfMinMaxes):
    return [ randomUniform(min, max) for [min, max] in arrayOfMinMaxes]

def ex():
    randomMultivariateGaussian([ (1.0, 10.0), [-5.0, 2.0], [6.0, .001]])
    return randomGaussian(0.0, 1.0)

# usage:
# [x for x in lowVarianceSample(["Common", "Uncommon"], [10.0, 1.0], 10)]
# draws newSize # of objects from objects with corresponding weights
# yields for each object
# credit: http://www.google.com/codesearch/p?hl=en&sa=N&cd=2&ct=rc#D6Z80snicPI/trunk/fvision_modules/prob/src/fvision/prob/resampling.cpp&q=low%20variance%20resampling
def lowVarianceSample2(objects, weights, newSize):
    if newSize > 0:
        sumWeights = float(sum(weights))
        weights = [float(x) / sumWeights for x in weights]
        print sum(weights)
        invSize = 1.0 / float(newSize)
        r = random.random() * invSize
        i = 0
        c = weights[0]
        for m in xrange(0, newSize):
            u = r + float(float(m) - 1.0 ) * invSize
            while u > c:
                i = i + 1
                c = c + weights[i]
            yield objects[i]

def lowVarianceSampleBuggy(objects, samples = None):
    if not samples or samples <= 0:
        if samples <= 0 or len(objects) == 0:
            raise Exception("Invalid number of samples provided.")
        samples = len(objects)
    rospy.loginfo("Num Samples: %d", samples)
    totalWeight = sum([o.weight for o in objects])
    result = []
    offset = random.random()*totalWeight/float(samples)
    increment = float(totalWeight)/float(samples)
    cumulativeWeight = objects[0].weight
    nextSample = offset
    index = 0
    for i in range(samples):
        while nextSample > cumulativeWeight:
            index += 1
            cumulativeWeight += objects[index].weight
        nextSample += increment
        result.append(objects[index])
    assert(samples == len(result))        
    for r in result:
        r.weight = 1.0 / float(len(result))
    return result
