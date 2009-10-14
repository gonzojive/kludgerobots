#!/usr/bin/env python
import roslib; roslib.load_manifest('hw1')
import rospy
import tf
from math import *
from line import *
from lineviz import *
from compass import *
from vector import *
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

r2d = 180.0/3.14159
d2r = 3.14159/180.0

#Given an angle and a radius, returns the X,Y coordinates of that point in cartesian coordinates
# this assumes the following:
# 1.  The cartesian coordinate is such that in front of the robot is positive X and the LEFT of
# the robot is positive Y
# 2.  theta = 0, r = 1 corresponds to (0, -1) and theta = pi/2, r=1 corresponds to (1, 0)
def polarToCartesian(r, theta):
  x = r * cos(theta)
  y = r * sin(theta)
  return [y, -x]


def angleDifferenceRadians(a1, a2):  # returns angle difference in the range [0,2pi]
  temp = a2-a1
  while temp > 2*pi:
    temp -= 2*pi
  while temp < 0:
    temp += 2*pi
#  rospy.loginfo("Angle Diff: a1=%0.2f, a2=%0.2f, diff=%0.2f", a1, a2, temp)
  return temp


# Given a bunch of laser readings
def laserReadingAngle(i, readingRanges):
  num_scan_points = len(readingRanges)
  if num_scan_points > 0:
    return (1.0 - float(i) / float(num_scan_points)) * pi
  else:
    return 0 #degenerate

def laserReadingToCartesianPoints(reading):
  return map(lambda rng,i: polarToCartesian(rng, laserReadingAngle(i, reading.ranges)), reading.ranges, xrange(0, len(reading.ranges)))

class LaserInterpreter:
  def __init__(self, p, c, cmd): # constructor
    self.compass = c
    self.position = p
    self.doRansac = 1
    self.mapviz = LocalMapVisualizer()
    self.maxAngleDiff = 10.0 / r2d #should be in radians 
    self.bestMasterInliers = []
    self.lastReading = 0

  def laserReadingNew(self, reading):
  # Do some processing on the new laser reading
    if self.doRansac >= 1:
      self.ransac(reading)

  def logReadingInfo(self, reading):
    rospy.loginfo("Last: %0.2f  Middle: %0.2f  First: %0.2f ", reading.ranges[-1], reading.ranges[len(reading.ranges)/2], reading.ranges[0])
    for r in reading.ranges:
      print r

  #realign ourselves against a particularly well-fit wall if necessary
  def maybeRealignAgainstWall(self, wallLine, inliers):
    if len(inliers) > len(self.bestMasterInliers):
       rospy.loginfo("Setting new master wall with %d inliers", len(inliers))
       # set the global compass to use this as the master
       self.compass.setMaster(wallLine)
       self.bestMasterInliers = inliers

  #function to perform ransac on the points and infer the heading
  def ransac(self, reading):
    cartesianPoints = laserReadingToCartesianPoints(reading)
#    self.mapviz.vizPoints(cartesianPoints)
#    rospy.loginfo("Starting RANSAC")
    [bestLine, inliers] = fitLineWithRansac(cartesianPoints, .015)
   
    angle = self.compass.getOrientation(bestLine)
    rospy.loginfo("Raw compass angle = %0.2f", angle*r2d)
    rospy.loginfo("Raw/Corrected odometry:  %0.2f  %0.2f", self.position.odomRot*r2d, self.position.rotation()*r2d)

    odomAngle = self.position.rotation()
    if(self.position.isTurning == True): 
      #update the angle only if we are turning, else just stay with current angle
      if abs(abs(angle) - abs(odomAngle)) < self.maxAngleDiff:  
        # original angle is good... just perform a no-op
        angle = angle
      #now make sure that differences in the direction of the ransac vector don't affect our heading
      elif abs(pi - abs(angle) - abs(odomAngle)) < self.maxAngleDiff:
        angle = abs(pi - angle)
      elif abs(pi/2.0 + abs(angle) - abs(odomAngle)) < self.maxAngleDiff:
        angle = pi/2.0 + angle
      elif abs(abs(angle) - pi/2.0 - abs(odomAngle)) < self.maxAngleDiff:
        angle = abs(angle - pi/2.0)
      else:
        # if we do not find a good wall match, just use the odometric rotational delta to correct
        # the previous corrected rotation
        angle = self.position.rotation()  # this will just keep the same offset as before
      # get the angle between 0 and 2pi
      if angle < 0:
         angle = angle + 2.0 * pi
      elif angle > 2.0 * pi:
         angle = angle - 2.0 * pi

      # ignore readings that contradict the odometry differences
      
      if self.position.odomRot > self.position.lastOdomRot and angle < self.position.rotation():
        angle = self.position.rotation()
      elif self.position.odomRot < self.position.lastOdomRot and angle > self.position.rotation():
        angle = self.position.rotation()
    else:
      #we are not turning, so the angle doesn't need to be updated
      angle = self.position.rotation()
    # so now angle holds the corrected angle estimate
    self.position.compassReading(angle)
    self.lastReading = angle
#    rospy.loginfo("Compass master wall: %0.2f, %0.2f", self.compass.master.trajectory[0], self.compass.master.trajectory[1])
    rospy.loginfo("Global Compass Rotation: = %0.2f",  self.position.rotation()*r2d)



NUMERIC_TOL = .000001
 
def normalizeAngle360(rads):
    
    if (rads > 2.0 * pi + NUMERIC_TOL):
        return normalizeAngle360(rads - (2.0 * pi))
 
    elif (rads < 0 - NUMERIC_TOL):
        return normalizeAngle360(rads + (2.0 * pi))
 
    else:
        return rads
 
# ensures a positive angle between 0 and 2pi
def normalizeAngle180(rads):
    rads = normalizeAngle360(rads)
    if (rads > pi):
        return 2 * pi - rads
    else:
        return rads
 
def normalizeAngle90(rads):
    rads = normalizeAngle360(rads)
    # > 180 degrees, reduce to the 180 sphere
    if (rads > pi + NUMERIC_TOL):
        return normalizeAngle90(2 * pi - rads)
    elif (rads > pi): # within tolerance
        return 0
    elif (rads > pi/2.0):
        return pi - rads
    else:
        return rads

class RobotPosition:
  def __init__(self):
    self.initialized = False  # can't start until we initialize odometry
    self.odomTrans = [0,0]  
    self.odomRot = 0
    self.offsetTrans = [0,0] # this is equal to the offset of 
    self.offsetRot = 0 # this is equal to our true rotation - the odometer's rotation
    self.lastOdomTrans = [0,0] # last reading of the odometer's XY
    self.lastOdomRot = 0 # last reading of the odometer's rotation
    self.isTurning = False
    self.odomRot0 = 0
  def resetOdom(self, t, r):  # resets the odometry offsets
    self.odomTrans0 = t
    self.odomRot0 = math.acos(r[3])*2
  def odomReadingNew(self, t, r):  # calculate a new odometry reading with respect to the offsets
    [self.lastOdomTrans[0], self.lastOdomTrans[1]] = self.odomTrans
    self.lastOdomRot = self.odomRot
    self.odomTrans[0] = t[0] - self.odomTrans0[0]
    self.odomTrans[1] = t[1] - self.odomTrans0[1]
    self.odomRot = math.acos(r[3])*2 - self.odomRot0
  # set the compass given a corrected rotation value
  def compassReading(self, correctedRotation):  # take in a new compass reading
    # offsetRot gives the difference between the robot's true rotation and what the odom tells us it is
    self.offsetRot = correctedRotation - self.odomRot  # make the offset take it to the real angle
  def position(self):  # returns the corrected position
      odomX = self.odomTrans[0]
      odomY = self.odomTrans[1]
      [odomX, odomY] = vector_rotate_2d([odomX, odomY] , -1.0 * self.odomRot0)
      return [self.offsetTrans[0]+odomX, self.offsetTrans[1]+odomY]
  def rotation(self):  # returns the corrected rotation
    return self.offsetRot + self.odomRot
 
#
#class to perform movements towards goal (origin in this case)
class TrackRoute:
    def __init__(self,robotPosition):
       
        self.rp = robotPosition  # the RobotPosition
        self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        self.stage = 1 # go through 4 stages: 1. go fwd 10 feet. 2. turn around. 3.
        self.viz = LocalMapVisualizer()
        self. vOrigin = rp.position() #the origin is wherever we are right now and the goal is the destn.

    def setGoal(self, goalXY):
        self.goal = goalXY
        self.paintGoal()
      
    def setOrigin(self,positionXY):
        self.vOrigin = positionXY

    def paintGoal(self):
        # figure out the vector to the goal
        vToGoal = vector_minus(self.goal, self.vOrigin)

        self.viz.vizPoints([vToGoal])

    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self,startOrEnd):
        if startOrEnd == 2:
          temp = self.goal
          self.goal = self.vOrigin
          self.vOrigin = temp
         # figure out the robot position        
        vForward = polarToCartesian(1.0, self.rp.rotation() + pi/2.0)
        rospy.loginfo(self.goal)
        rospy.loginfo(self.vOrigin)
        # figure out the vector to the goal
        vToGoal = vector_minus(self.goal, self.vOrigin)
        self.viz.vizSegment([0,0,0], vToGoal)#, the_id=1)
        self.viz.vizSegment([0,0,0], vForward)#, the_id=2)
        if(vToGoal == [0,0]):
          return 1
     
    
        # determine the angle between the the forward vector and the v
        signedAngleToGoal = vector_angle_signed(vForward, vToGoal)

        MAX_ANGULAR_VELOCITY = 20.0 *d2r
        MAX_LINEAR_VELOCITY = .50 # 50 cm

        angularVelocity = 0
        linearVelocity = Vector3(0,0,0)
        
        # if the angle is off by more than 5 degrees, just rotate
        if math.fabs(signedAngleToGoal) > d2r*(80.0) and normalizeAngle90(signedAngleToGoal) > d2r*(80.0):
            angularVelocity = signedAngleToGoal
            if math.fabs(signedAngleToGoal) > MAX_ANGULAR_VELOCITY:
               sign = (signedAngleToGoal >= 0 and 1.0) or -1.0
               angularVelocity = MAX_ANGULAR_VELOCITY * sign  
        elif vector_length_squared(vToGoal) > .05:
            vVel = vToGoal
            if vector_length_squared(vVel) > MAX_LINEAR_VELOCITY:
                vVel = vector_scale(normalize(vVel), MAX_LINEAR_VELOCITY)

            rospy.loginfo("Setting velocity to [%0.2f, %0.2f]", vVel[0], vVel[1])
            linearVelocity = Vector3(vVel[0],vVel[1],0)
            
        # otherwise if the goal is kind of far away, set the forward velocity
        self.velPublish.publish(Twist(linearVelocity,Vector3(0,0,angularVelocity)))
        #return [linearVelocity,angularVelocity]
        return 0



class Commands:
  def __init__(self, rp):
    self.xGoal = 10                                       # 4m
    self.thetaGoal = 180 * d2r                             #this angle has to be in radians!
    self.rp = rp                                         # the RobotPosition
    self.velPublish = rospy.Publisher("commands", Twist) #publish to commands
    self.stage = 1
    self.turning = 0
    self.pathfinder = TrackRoute(rp)
     #some constants
    self.STRAIGHT_SPEED = 0.5
    self.SLOW_STRAIGHT_SPEED = 0.2
    self.REALLY_SLOW_TURN_SPEED = 2.0 * d2r
    self.SLOW_TURN_SPEED = 5.0 * d2r
    self.FAST_TURN_SPEED = 20.0 *d2r
    self.DISTANCE_BEFORE_TURN = 2.0
  
  def orientCorrectly(self,rp,startOrEnd):
    self.pathfinder.setOrigin(rp.position())
    angle = rp.rotation()
    if startOrEnd == 1: #we are finding a route to the destination
      self.startPosition = rp.position()
      self.pathfinder.setGoal ([rp.odomTrans0[0]+self.xGoal,rp.odomTrans0[1]])
    else:
      self.pathfinder.setGoal([self.startPosition[0]-self.xGoal,self.startPosition[1]]) # go back to starting point
    return self.pathfinder.publishNextMovement(startOrEnd)

  def performPartA(self):
    return_angle = 180.0 *d2r
    start_angle = 0.0
    ''' stages in the operation:
      1. Head straight to the turning point
      2. Turn quickly till we near 180 deg
      3. Turn slowly as we approach the target angle
      4. Correct for any angular errors during the turn using laser and head back to starting position
      rospy.loginfo("return angle: %f",return_angle)
    '''
    xVel = Vector3(0.0,0.0,0.0)
    thetaVel = 0
    laserTrans = rp.position() # current position
    laserRot = rp.rotation() # current rotation
    if self.stage == 1:
      rospy.loginfo("in stage 1: lasertrans[0]=%f,xGoal=%f",laserTrans[0],self.xGoal)
      #go to turning point
      x = self.orientCorrectly(rp,1)
      if laserTrans[0] >= self.xGoal-0.2 or x == 1:
        self.stage += 1
        
    elif self.stage == 2:
      rospy.loginfo("in stage 2")
      #turn
      xVel = Vector3(0.0,0.0,0.0)
      self.turning = 1
      thetaVel = self.FAST_TURN_SPEED
      rospy.loginfo("thetaVel =   rp.resetOdom(trans, rot)%f",thetaVel)  
      twist = Twist(xVel,Vector3(0,0,thetaVel))
      self.velPublish.publish(twist)
      if laserRot >= self.thetaGoal - 1: #turn relatively fast till we are close to the goal
          self.stage += 1
          
    elif self.stage == 3:
      #turn slowly till 180
      rospy.loginfo("in stage 3")
      self.turning = 1
      thetaVel = self.SLOW_TURN_SPEED
      rospy.loginfo("thetaVel = %f",thetaVel)  
      twist = Twist(xVel,Vector3(0,0,thetaVel))
      self.velPublish.publish(twist)
      if laserRot >= self.thetaGoal - 0.01: #turn slowly as we reach the goal
          self.stage += 1
   
    elif self.stage == 4:
      #go back to origin
      rospy.loginfo("in stage 4")
      self.turning = 0
      x = self.orientCorrectly(rp,2)
    if self.turning ==1 or x==1:
      rp.isTurning = True
    else:
      rp.isTurning = False
    
    

  
compass = Compass()
rp = RobotPosition() # global RobotPosition object
cmd = Commands(rp)
li = LaserInterpreter(rp, compass, cmd)  # global LaserInterpreter object


def callback(reading):
#  rospy.loginfo("Laser reading number %d", reading.header.seq)
  li.laserReadingNew(reading)


def init_node():
  rospy.init_node('kludge1_1')
  rospy.loginfo('"KLUDGE 1.1" node is awake')
  rospy.Subscriber("laser", LaserScan, callback, queue_size = 1) # listen to "laser"
  rospy.loginfo("Subscribed to laser readings")
  odoListener = tf.TransformListener() # listen to tf
  rospy.loginfo("Subscribed to odometry frames")
  rate = rospy.Rate(10) # 10 Hz

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
    if compass.initialized:
      cmd.performPartA()
    rate.sleep()

if __name__ == '__main__':
  try:
    init_node()
  except rospy.ROSInterruptException:
    pass
