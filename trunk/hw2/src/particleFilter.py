import rospy
import pose
import threading
import util
import statutil
import motionModel
import math
import mapManager
import mapmodel
import geometry_msgs
import geometry_msgs.msg
# Minimum values before we run the filter again
minimumMovement = .03    # 30 mm - probably want to increase this when we have more samples
minimumTurn = util.d2r(10)  # 10 degrees


class ParticleFilter(threading.Thread):
    def __init__(self, odom, viz, motionErr, transformer, initialPose):
        threading.Thread.__init__(self) # initialize the threading package
        self.transformer = transformer
        self.lastOdom = odom[0] + [odom[1]] # convert from [[x, y], a] to [x, y, a]
        self.newOdom = self.lastOdom[:] # makes a deep copy
        self.motionError = motionErr or motionModel.MotionErrorModel()   # with no arguments, we should get 0 variance
        self.runFilter = 0  # don't run the filter until you've moved enough
        self.runFilterLock = threading.Lock()
        self.poseAverage = pose.Pose(0, 0, 0)
        self.poseSet = pose.PoseSet(viz, 50)    # 50 poses just for testing purposes
        #self._pFilter.poseSet.initializeUniformStochastic( [-1, 1], [-1, 1], [0, 2*math.pi] )
        self.poseSet.initializeGaussian( [initialPose[0], .5], [initialPose[1], 0.5], [initialPose[2], math.pi/2.0] )
        self.mapManager = mapManager.MapManager(initialPose)
        self.startTime = rospy.Time.now()
        self.mapModel = mapmodel.MapModel()

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
        self.testCaster()
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
            self.runFilterLock.acquire()    # <---grab the lock--->
            motion = motionModel.odomToMotionModel(self.lastOdom, self.newOdom) # get the motion model
            self.lastOdom = self.newOdom[:]  # save this new odometry
            self.runFilter = 0  # reset the filter flag
            self.runFilterLock.release()    #  <---release the lock--->
            self.startTime = rospy.Time.now()
            self.predictionStep(motion)
            self.updateMapTf()
            self.displayPoses() # poses have changed, so draw the new ones

    def updateMapTf(self):
        self.updatePoseAverage()
        #self.mapManager.updateMapToOdomTf(self.poseAverage, self.lastOdom, self.startTime)

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
    # I think this is a valid assumption in our program, but an alternative is to create a lock object in
    # the Pose class, so each pose can be locked individually. Are locks expensive? I don't think they are.
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
        if self.mapManager:    # if we have a valid map, cull the poses that are in illegal positions
            self.poseSet.poses = filter(lambda p: self.mapManager.inBounds(p), self.poseSet.poses)
