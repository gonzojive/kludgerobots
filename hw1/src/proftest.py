import cProfile
import random
from line import *

points = map(lambda x: [random.random() * 10.0, random.random() * 100.0], xrange(0, 160))

def testFn():
    for i in xrange(0, 1):
        fitLineWithRansac(points, .05)
