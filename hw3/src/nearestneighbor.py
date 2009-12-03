from vector import *

def nearestNeighbor(collection, pt):
    minDistSqrd = None
    best = None
    for (index, point) in enumerate(collection):
        distSqrd = vector_length_squared(vector_minus(point, pt))
        if not best or minDistSqrd > distSqrd:
            minDistSqrd = distSqrd
            best = (point, index, minDistSqrd)
    return best
