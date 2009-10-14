#!/usr/bin/env python
import roslib; roslib.load_manifest('hw1')
import rospy
import tf
from math import *
from line import *
from localMap import *
from lineviz import *
# import cv	# doesn't recognize cv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

r2d = 180.0/3.14159

#Given an angle and a radius, returns the X,Y coordinates of that point in cartesian coordinates
# this assumes the following:
# 1.  The cartesian coordinate is such that in front of the robot is positive X and the LEFT of
# the robot is positive Y
# 2.  theta = 0, r = 1 corresponds to (0, -1) and theta = pi/2, r=1 corresponds to (1, 0)
def polarToCartesian(r, theta):
  x = r * cos(theta)
  y = r * sin(theta)
  return [y, -x]

# Given a bunch of laser readings
def laserReadingAngle(i, readingRanges):
  num_scan_points = len(readingRanges)
  if num_scan_points > 0:
    return (1.0 - float(i) / float(num_scan_points)) * pi
  else:
    return 0 #degenerate

def laserReadingToCartesianPoints(reading, position):
  pos = position.position()
  local = map(lambda rng,i: polarToCartesian(rng, laserReadingAngle(i, reading.ranges)+pos[1]), reading.ranges, xrange(0, len(reading.ranges)))
  return [[p[0]-pos[0][0],p[1]-pos[0][1]] for p in local]

class LaserInterpreter:
  def __init__(self, p, m): # constructor
    self.localMap = m
    self.position = p
    self.readings = []
    self.maxReadings = 10
    self.R = 100  # number of subdivisions of radius
    self.T = 100  # number of subdivisions of theta
    self.maxRadius = 11.0  # laser only accurate to 12 feet, let's say 11 to be safe
    self.maxTheta = 3.14159  # theta in [0, pi]
    self.radiusInc = self.maxRadius / float(self.R)  # step size of radius
    self.thetaInc = self.maxTheta / float(self.T)  # step size of theta
    self.radiusValues = []  # start with empty arrays
    self.thetaValues = []
    self.doHough = 0  # determines whether to perform Hough or not
    self.doRansac = 1
    self.mapviz = LocalMapVisualizer()
    cur = 0.0
    for i in range(self.R):  # fill in the radius bins
      self.radiusValues.append(cur)
      cur += self.radiusInc
    cur = 0.0
    for i in range(self.T):  # fill in the theta bins
      self.thetaValues.append(cur)
      cur += self.thetaInc
    self.A = [self.T*[0] for i in range(self.R)]  # make the R x T accumulator array

  def laserReadingNew(self, reading):
    rospy.loginfo('Laser reading received...')  
    # Do some processing on the new laser reading
    self.readings.append(reading)
    if len(self.readings) > self.maxReadings:
      self.readings.pop(0)	# remove the oldest reading
    if self.doHough >= 1:
      self.hough(reading)
      self.doHough = 0
    if self.doRansac >= 1:
      self.ransac(reading)
      self.doRansac = 1
    
    rospy.loginfo("Printing %d walls." % len(localMap.walls))
    for w in self.localMap.walls:
      [begin, end] = w.segment()
      self.mapviz.vizSegment(begin, end)
      rospy.loginfo("Wall: (%0.2f, %0.2f) to (%0.2f, %0.2f)", begin[0], begin[1], end[0], end[1])
#      rospy.loginfo("Extremes: %0.2f %0.2f %0.2f %0.2f", w.leftmost, w.rightmost, w.bottommost, w.topmost)
      
#    self.logReadingInfo(reading)

  def logReadingInfo(self, reading):
    rospy.loginfo("Min: %d  Max: %d  Inc: %f  Len: %d", r2d*reading.angle_min, r2d*reading.angle_max, r2d*reading.angle_increment, len(reading.ranges))
    print reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0]

  def ransac(self, reading):
    cartesianPoints = laserReadingToCartesianPoints(reading, self.position)
    [bestLine, inliers, extremes] = fitLineWithRansac(cartesianPoints, .03)
#    rospy.loginfo("Extremes: %0.2f %0.2f %0.2f %0.2f", extremes[0], extremes[1], extremes[2], extremes[3])
    self.mapviz.vizPoints(inliers)
    #for pt in cartesianPoints:
    #rospy.loginfo("(%0.2f, %0.2f)", pt[0], pt[1])
    def s(arr):
      return "[%0.2f, %0.2f]" % (arr[0], arr[1])
    rospy.loginfo("Line: %s trajectory: %s" % (s(bestLine.origin),  s(bestLine.trajectory)))
#    rospy.loginfo("  %i Inliers of %i readings: %s" % (len(inliers), len(reading.ranges), map(s, inliers)))
    angleDiff = self.localMap.wallIs(bestLine, extremes)
    if angleDiff != None:
      self.position.addMapRotationOffset(angleDiff)


class RobotPosition:
  def __init__(self):
    self.initialized = False  # can't start until we initialize odometry
    self.odomTrans = [0,0]  
    self.odomRot = 0
    self.mapTrans = [0,0]
    self.mapRot = 0
  def resetOdom(self, t, r):  # resets the odometry offsets to the current odometry value
    self.odomTrans0 = t
    self.odomRot0 = r[2]
  def odomReadingNew(self, t, r):  # calculate a new odometry reading with respect to the offsets
    self.odomTrans[0] = t[0] - self.odomTrans0[0]
    self.odomTrans[1] = t[1] - self.odomTrans0[1]
    self.odomRot = (r[2] - self.odomRot0) * 180.0
  def mapPositionNew(self, mapT, mapR):  # take in a new map reading
    self.mapTrans = mapT  # assume map tranlation and rotation are base truth
    self.mapRot = mapR
    self.resetOdom(self.odomTrans, self.odomRot)  # set odometry offsets back to 0
  def addMapRotationOffset(self, offset):  # just offset the current reading
    self.mapRot += offset
  def position(self):  # returns the most recent map position + any more recent odometry offsets
    return [[self.mapTrans[0]+self.odomTrans[0], self.mapTrans[1]+self.odomTrans[1]], self.mapRot+self.odomRot]
#    self.logPosInfo()
  def logPosInfo(self):
    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f degrees", self.trans[0], self.trans[1], self.rot)
    

compass = Compass()

class Commands:
  def __init__(self, rp):
    self.xGoal = 4  # 4m
    self.thetaGoal = 180  # 180 degrees
    self.rp = rp  # the RobotPosition
    self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
    self.stage = 1 # go through 4 stages: 1. go fwd 10 feet. 2. turn around. 3. 
  def send(self):
    self.velPublish.publish(Twist(Vector3(0,0,0),Vector3(0,0,4.0/r2d)))
    '''    
    xVel = 0
    thetaVel = 0
    if self.stage == 1:  # haven't gone forward enough
      xVel = 0.5
      if rp.trans[0] >= self.xGoal:
        self.stage += 1
    elif self.stage == 2:  # haven't turned enough
      thetaVel = 10.0/r2d  # turn 10 degrees (per second?)
      if rp.rot >= self.thetaGoal - 1:
        self.stage += 1
    elif self.stage == 3:
      thetaVel = 1.0/r2d
      if rp.rot >= self.thetaGoal - 0.01:
        self.stage += 1
    elif self.stage == 4:
      xVel = 0.5
      if rp.trans[0] <= 0:
        self.stage += 1
    twist = Twist(Vector3(xVel, 0, 0), Vector3(0, 0, thetaVel))
    self.velPublish.publish(twist)
    '''
    

localMap = LocalMap() # the map object
rp = RobotPosition() # global RobotPosition object
li = LaserInterpreter(rp, localMap)	# global LaserInterpreter object


def callback(reading):
  li.laserReadingNew(reading)

def init_node():
  rospy.init_node('kludge1_1')
  rospy.loginfo('"KLUDGE 1.1" node is awake')
  rospy.Subscriber("laser", LaserScan, callback) # listen to "laser"
  rospy.loginfo("Subscribed to laser readings")
  
  odoListener = tf.TransformListener() # listen to tf
  rospy.loginfo("Subscribed to odometry frames")
  rate = rospy.Rate(10.0) # 10 Hz
  cmd = Commands(rp)
  while not rp.initialized:
    try:
      (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
      rp.initialized = True
    except (tf.LookupException, tf.ConnectivityException):
      continue
  rp.resetOdom(trans, rot)
  while not rospy.is_shutdown():
    try:
      (trans, rot) = odoListener.lookupTransform('/odom', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
      continue
#    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f", trans[0], trans[1], rot[2])
    rp.odomReadingNew(trans, rot)
#    cmd.send()
    rate.sleep()

if __name__ == '__main__':
  try:
    init_node()
  except rospy.ROSInterruptException:
    pass
