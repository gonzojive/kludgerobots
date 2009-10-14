import math
import vector
import ransac
from ransac import *
from vector import *

FP_TOLERANCE = .000001

# an infinite line
class LineModel:
    #given two points initialize the line
    def __init__(self, two_cartesian_points):
        [a, b] = two_cartesian_points
        a_to_b = map(lambda acoord, bcoord: bcoord - acoord, a, b)
        self.origin = a
        self.trajectory = normalize(a_to_b)
	if self.trajectory[1] < 0:
	   self.trajectory = vector_scale(self.trajectory, -1.0)

    #returns the angle between two lines
    def angleBetween(self, anotherLineModel):
        return vector_angle(self.trajectory, anotherLineModel.trajectory)

    #returns the minimum distance between the line and the point
    def distanceToPoint(self, pt):
        w = vector_minus(pt, self.origin)
        traj_cross_w = cross(self.trajectory, w)
        return vector_length(traj_cross_w) / vector_length(self.trajectory)

    def distanceToPointSquared(self, pt):
        w = vector_minus(pt, self.origin)
        traj_cross_w = cross(self.trajectory, w)
        return vector_length_squared(traj_cross_w) / vector_length_squared(self.trajectory)

    #returns the intersection of two lines
    def intersection(self, anotherLineModel):
        denom = -self.trajectory[0]*anotherLineModel.trajectory[1]+anotherLineModel.trajectory[0]*self.trajectory[1]
        if math.fabs(denom) < FP_TOLERANCE:
          return None
        t = 1.0/denom
        t *= (-anotherLineModel.trajectory[1]*(anotherLineModel.origin[0]-self.origin[0])+anotherLineModel.trajectory[0]*(anotherLineModel.origin[1]-self.origin[0]))
        return [self.origin[0]+t*self.trajectory[0], self.origin[1]+t*self.trajectory[1]]

    def point(self, t):
      return [self.origin[0]+t*self.trajectory[0], self.origin[1]+t*self.trajectory[1]]


def fitLineWithRansac(points, distance_cutoff):
    [best_model, best_inliers] = ransac(points, 2, LineModel, lambda model, pt: model.distanceToPointSquared(pt) < (distance_cutoff * distance_cutoff))
    return [best_model, best_inliers]

    
    
    
