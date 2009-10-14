from line import lineModel
import math

class Wall:
  def __init__(self, line, extremes):
    self.lineModel = line
    self.polar = [line.distanceToPoint([0,0]), math.atan2(line.trajectory[1], line.trajectory[0])]
    self.leftmost = extremes[0]
    self.rightmost = extremes[1]
    self.bottommost = extremes[2]
    self.topmost = extremes[3]
    self.confidence = 1
  def angleBetween(self, wall):
    return self.lineModel.angleBetween(wall.lineModel)
  def distanceToPoint(self, point):
    return self.lineModel.distanceToPoint(point)
  def origin(self):
    return self.lineModel.origin
  def trajectory(self):
    return self.lineModel.trajectory
  def segment(self):
    if abs(self.lineModel.trajectory[0]) > abs(self.lineModel.trajectory[1]):  # more horizontal than vertical
      t1 = (self.leftmost - self.lineModel.origin[0])/self.lineModel.trajectory[0]
      t2 = (self.rightmost - self.lineModel.origin[0])/self.lineModel.trajectory[0]
    else:  # more vertical than horizontal
      t1 = (self.bottommost - self.lineModel.origin[1])/self.lineModel.trajectory[1]
      t2 = (self.topmost - self.lineModel.origin[1])/self.lineModel.trajectory[1]
    origin = self.lineModel.point(t1)
    terminus = self.lineModel.point(t2)
    return [origin, terminus]
    

class LocalMap:
  def __init__(self):
    self.walls = []  # list of walls
    self.maxAngleDiff = 5.0 * math.pi / 180.0  # the maximum degree difference to be the same wall
    self.maxDistDiff = 0.4  # the maximum distance apart to be the same wall

  def wallIs(self, line, extremes):
    newWall = Wall(line, extremes)
    [i, angle] = self.wallIndex(newWall)  # returns an index if close enough, -1 otherwise
    if i < 0:  # no similar wall found
      if self.isRectilinear(newWall):  # this one will work
        self.walls.append(newWall)
      return None
    else:  # similar wall was found
      self.walls[i].confidence += 1  # add confidence value
      return angle

  def wallIndex(self, w):  # takes in a new wall, outputs the index of the most similar existing wall, or -1 if none
    minAngle = self.maxAngleDiff  # make sure we have at most this much angle diff
    minDist = self.maxDistDiff  # at most this much distance diff
    retval = -1  # retval of -1 means no match
    for wall in self.walls:  # iterate over all the walls
      angle = wall.angleBetween(w)  # calculate the difference angle
      if angle > math.pi - self.maxAngleDiff:  # close to pi is the same as close to 0
        angle = angle - math.pi  # make the angle negative so we know which way to correct odom
      if angle < minAngle and wall.distanceToPoint(w.origin()) < minDist:  # smallest differences yet
        # check for any separation between the new wall and the old one
        if not (w.leftmost > wall.rightmost or w.rightmost < wall.leftmost or w.bottommost > wall.topmost or w.topmost < wall.bottommost):
          minAngle = angle  # update the minAngle so we have to do better to overwrite this
          retval = self.walls.index(wall)
    if retval >= 0:  # if we found a match, update extremes
      if w.leftmost < self.walls[retval].leftmost:
        self.walls[retval].leftmost = w.leftmost
      if w.rightmost > self.walls[retval].rightmost:
        self.walls[retval].rightmost = w.rightmost
      if w.bottommost < self.walls[retval].bottommost:
        self.walls[retval].bottommost = w.bottommost
      if w.topmost > self.walls[retval].topmost:
        self.walls[retval].topmost = w.topmost
    return [retval, minAngle]

  def isRectilinear(self, w):  # takes in a new wall, makes sure it sits at either 90 or 180 degrees to other walls
    if len(self.walls) == 0:
      return True
    angleTests = [0, math.pi/2.0, math.pi, 3.0*math.pi/2.0, 2*math.pi]  # test at all 90 degree intervals
    for wall in self.walls:
      a = wall.angleBetween(w)
      for test in angleTests:
        if a > test-self.maxAngleDiff and a < test+self.maxAngleDiff:
          return True
    return False
