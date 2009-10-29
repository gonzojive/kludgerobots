import rospy
import pose
import threading
import util
import statutil
import motionModel
import math
import mapmodel
import geometry_msgs
import geometry_msgs.msg
import laser
import random

from vector import *

# Minimum values before we run the filter again
minimumMovement = .03    # 30 mm - probably want to increase this when we have more samples
minimumTurn = util.d2r(10)  # 10 degrees


class ParticleFilter(threading.Thread):
    # constructor takes:
    # odom - the position of the robot in the odometry frame according to the odometry
    # viz - a vizualizer instance for dipslaying stuff in rviz
    # laser - an instances of Laser from laser.py
    # motionErr - and instance of MotionErrorModel
    # transformer - the odometry object that we use to transform between frames
    # initialPose - the initial guess of the robot in the Map frame (an instance of Pose)
    def __init__(self, odom, viz, laser, motionErr, transformer, initialPose):
        threading.Thread.__init__(self) # initialize the threading package
        self.transformer = transformer
        # lastOdom stores
        self.lastOdom = odom[0] + [odom[1]] # convert from [[x, y], a] to [x, y, a]
        self.newOdom = self.lastOdom[:] # makes a deep copy
        self.laser = laser
        self.motionError = motionErr or motionModel.MotionErrorModel()   # with no arguments, we should get 0 variance
        self.runFilter = 0  # don't run the filter until you've moved enough
        self.runFilterLock = threading.Lock()
        self.poseAverage = pose.Pose(0, 0, 0)
        self.numDesiredPoses = 50 # used during resampling
        self.poseSet = pose.PoseSet(viz, self.numDesiredPoses)    # 50 poses just for testing purposes
        #self._pFilter.poseSet.initializeUniformStochastic( [-1, 1], [-1, 1], [0, 2*math.pi] )
        self.poseSet.initializeGaussian( [initialPose.x, .5], [initialPose.y, 0.5], [initialPose.theta, math.pi/4.0] )
        self.startTime = rospy.Time.now()
        self.mapModel = mapmodel.MapModel(initialPose)


    # displayPoses(): draws the current poseSet to rviz
    # may exhibit odd behavior while the filter is updating poses; see pose.py
    def displayPoses(self):
        self.poseSet.display()
        self.poseSet.displayOne(self.poseAverage, displayColor = [0, 0, 1])

    # given a point in the robot frame, returns a point in the map frame
    def robotToMapFrame(self, pt):
        msg = geometry_msgs.msg.PointStamped()
        msg.header.frame_id = "base_laser"
        msg.point.x = pt[0]
        msg.point.y = pt[1]
        msg.point.z = pt[2]
        msg_in_my_frame = self.transformer.transformPoint("map", msg)
        ptOut = msg_in_my_frame.point
        return [ptOut.x, ptOut.y, ptOut.z]

    def testCaster(self):
        if self.mapModel.initializedp():
            # create a vector from the perspective of the robot
            robotOrigin = self.robotToMapFrame([1.0, 0.0, 0.0])
            rospy.loginfo("0,0 in map frame: [%0.2f, %0.2f]", robotOrigin[0], robotOrigin[1])
            #rospy.loginfo("Casting ray forard:Motion: %s", predict.toStr())  

  
    # receiveOdom(): updates the particle filter with the newest odometry reading
    # if the new reading is different enough, run the filter
    # parameters:
    #   odom -- the new odometry reading, currently in [[x, y], angle] format
    def receiveOdom(self, odom):
        #self.testCaster()
        self.runFilterLock.acquire()    # <---grab the lock--->
        self.newOdom = odom[0] + [odom[1]]  # convert from [[x, y], a] to [x, y, a]
        if self.runFilter == 0:
            manhattanDist = abs(self.newOdom[0]-self.lastOdom[0])+abs(self.newOdom[1]-self.lastOdom[1])
            angleDiff = util.smallestAngleDifference(self.newOdom[2], self.lastOdom[2])
            if manhattanDist >= minimumMovement or angleDiff >= minimumTurn:
                self.runFilter = 1  # run the filter next time the thread opens up
        self.runFilterLock.release()    # <---release the lock--->

    # run(): the main threading call
    # does a busy wait until needed, which is wasteful, but the alternative is to use events, where the danger is
    # that the program can't continuously check for rospy.is_shutdown(), and may never stop executing if the main
    # thread hits an error and unexpectedly quits. This is much safer.
    def run(self):
        while not rospy.is_shutdown():    # loop for the duration of the program
            # IMPORTANT: Do we need a lock to compare a value to 0? I don't think so, but not positive
            # a lock would slow this down a lot, and as long as the value can't go from 1 -> 0, I think we're OK
            if self.runFilter == 0: # no need to run the filter
                pass    # pass execution to another thread
                continue    # when this resumes, start at the top of the thread again
            # if we get here, we're going to run the filter
            self.runFilterLock.acquire()    # <---grab the odom lock--->
            self.laser.readingLock.acquire()    # <--- grab the laser lock --->
            self.startTime = rospy.Time.now()

            # collect data
            motion = motionModel.odomToMotionModel(self.lastOdom, self.newOdom) # get the motion model
            laserScan = self.laser.latestReading    # get the laser readings

            self.lastOdom = self.newOdom[:]  # save this new odometry
            self.runFilter = 0  # reset the filter flag
            self.laser.readingLock.release()    # <--- release the laser lock --->
            self.runFilterLock.release()    #  <---release the odom lock--->

            # 1.  Prediction step.  Use the assumed motion of the robot to update
            # the particle cloud.
            self.predictionStep(motion)
            # 2. Localize ourselves (update step) given some sensor readings
            #self.updateStep(laserScan)
            self.resampleStep()
            # tell the world where this particle filter thinks we are
            self.updateMapTf()
            rospy.loginfo("Updated the poses, displaying them now")
            self.displayPoses() # poses have changed, so draw the new ones

    def updateMapTf(self):
        self.updatePoseAverage()
        self.mapModel.updateMapToOdomTf(self.poseAverage, self.lastOdom, self.startTime)

    def updatePoseAverage(self):
        self.poseAverage = pose.Pose(0,0,0)
        thetaX = 0
        thetaY = 0
        totalWeight = 0
        for p in self.poseSet.poses:
            self.poseAverage.x += p.x*p.weight
            self.poseAverage.y += p.y*p.weight
            thetaX += math.cos(p.theta)
            thetaY += math.sin(p.theta)
            totalWeight += p.weight
        if totalWeight < util.NUMERIC_TOL:
            self.poseAverage.x = 0
            self.poseAverage.y = 0
            self.poseAverage.theta = 0
            self.poseAverage.weight = 0
        else:
            denom = 1.0/totalWeight
            self.poseAverage.x *= denom
            self.poseAverage.y *= denom
            self.poseAverage.theta = math.atan2(thetaY*denom, thetaX*denom)
            self.poseAverage.weight = 1

    # predictionStep(): update each pose given old and new odometry readings
    # this function currently does no locking, so it assumes that no other thread is modifying poseSet.
    # I think this is a valid assumption in our program, but an alternative is to create a lock object
    # in the Pose class, so each pose can be locked individually. Are locks expensive? I don't think
    # they are.
    # parameters:
    #   odomInitial --  an [x, y, angle] list of the previous odometry reading
    #   odomFinal   --  an [x, y, angle] list of the most recent odometry reading
    def predictionStep(self, motion):
        for p in self.poseSet.poses:
            predict = motionModel.Motion(0,0,0)
            # calculate the error in each movement, and make a new estimated motion
            # don't really want to take a square root every time here, can we just use the variance?
            predict.theta1 = statutil.randomGaussian(motion.theta1, math.sqrt(self.motionError.theta1Variance(motion)))
            predict.delta = statutil.randomGaussian(motion.delta, math.sqrt(self.motionError.deltaVariance(motion)))
            predict.theta2 = statutil.randomGaussian(motion.theta2, math.sqrt(self.motionError.theta2Variance(motion)))
            # convert back to odometry pose
            [dx, dy, dtheta] = motionModel.motionModelToOdom(predict)
            #rospy.loginfo("Motion: %s", predict.toStr())
            #rospy.loginfo("Pose0: %s", p.toStr())
            sinTheta = math.sin(p.theta)    # sin of the current pose angle
            cosTheta = math.cos(p.theta)    # cos of the current pose angle
            p.x += dx*cosTheta - dy*sinTheta    # rotate dx into pose frame and add
            p.y += dx*sinTheta + dy*cosTheta    # rotate dy into pose frame and add
            p.theta = util.normalizeAngle360(p.theta + dtheta)   # theta adds linearly
            #rospy.loginfo("Pose1: %s", p.toStr())
            self.cullIllegalPoses()

    
    def updateStep(self, laserScan):
        # pseudocode:
        # for all poses in particle filter:
        #    transform the laser scans into that pose frame
        #    calculate P(sensor reading | pose)
        #       - sensor reading may consist of many laser beam readings which are multiplied together
        #    P( sensor reading | pose) becomes the new weight for the particle
        #    maybe normalize the weights
        for pose in self.poseSet.poses:
            pSensorReadingGivenPose = self.calculateSensorReadingGivenPose(laserScan, pose)
            pose.weight = pSensorReadingGivenPose
        self.normalizeWeights()

    def resampleStep(self):
        for p in self.poseSet.poses:
            rospy.loginfo("Pose weight: %f", p.weight)
        self.poseSet.poses = statutil.lowVarianceSample(self.poseSet.poses, self.numDesiredPoses)
        if len(self.poseSet.poses) != self.numDesiredPoses:
            raise Exception("Not cool--sampling did not return requested number of objects.  Got back %i" % len(self.poseSet.poses))

    def calculateSensorReadingGivenPose(self, laserScan, pose):
        # pseudocode:
        # for each laser beam in the laser scan, calculate the probabability
        laserBeamVectors = laser.laserScanToVectors(laserScan)
        pLaserBeamsGivenPose = [self.pLaserBeamGivenPose(vLaserBeam, pose) for vLaserBeam in laserBeamVectors]
        product = 1.0
        n = 0
        for p in pLaserBeamsGivenPose:
            product *= p
            n += 1
            rospy.loginfo("product %i = %f * %f", n, product, p)
            if n > 2:
                break
        return product

    def pLaserBeamGivenPose(self, vLaserBeam, pose):
        vLaserBeamInMapFrame = pose.inMapFrame(vLaserBeam)
        d = self.mapModel.distanceFromObstacleAtPoint(vLaserBeamInMapFrame)
        stdDev = 1.34
        pGauss = statutil.gaussianProbability(0, stdDev, d)
        pUniform = 1.0/12.0
        weightGauss = .1
        weightUniform = .9

        rospy.loginfo("pGauss: %f", pGauss)

        return weightGauss*pGauss + weightUniform*pUniform
        
    def cullIllegalPoses(self):
        if self.mapModel:    # if we have a valid map, cull the poses that are in illegal positions
            self.poseSet.poses = filter(lambda p: self.mapModel.inBounds(p), self.poseSet.poses)

    def normalizeWeights(self):
        s = sum([p.weight for p in self.poseSet.poses])
        for pose in self.poseSet.poses:
            pose.weight = pose.weight / s
