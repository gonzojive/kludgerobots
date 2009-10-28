import math
import random

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