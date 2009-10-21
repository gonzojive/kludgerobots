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
    result = []
    for [mean, standardDeviation] in arrayOfMeansAndSigmas:
        result += [randomGaussian(mean, standardDeviation)]
    return result
    
                                                  
def ex():
    return randomGaussian(0.0, 1.0)
