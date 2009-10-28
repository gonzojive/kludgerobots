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

def lowVarianceSample(objects, samples = None):
    if not samples:
        samples = len(objects)
    rospy.loginfo("Num Samples: %d", samples)
    totalWeight = sum([o.weight for o in objects])
    result = []
    offset = random.random()/samples*totalWeight
    increment = float(totalWeight)/float(samples)
    cumulativeWeight = objects[0].weight
    nextSample = offset
    index = 0
    print offset, increment
    for i in range(samples):
        while nextSample > cumulativeWeight:
            index += 1
            cumulativeWeight += objects[index].weight
        nextSample += increment
        result.append(objects[index])
    for r in result:
        r.weight = 1
    return result
