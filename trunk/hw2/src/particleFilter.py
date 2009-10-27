import rospy
import pose
import threading
import util
import statutil
import motionModel
import math

# Minimum values before we run the filter again
minimumMovement = .03    # 30 mm - probably want to decrease this when we have more samples
minimumTurn = util.d2r(10)  # 10 degrees


class ParticleFilter(threading.Thread):
    def __init__(self, odom, viz, err = None, fm = None):
        threading.Thread.__init__(self) # initialize the threading package
        self.poseSet = pose.PoseSet(viz, 30)    # 30 poses just for testing purposes
        self.lastOdom = odom[0] + [odom[1]] # convert from [[x, y], a] to [x, y, a]
        self.newOdom = self.lastOdom[:] # makes a deep copy
        self.motionError = err or motionModel.MotionErrorModel()   # with no arguments, we should get 0 variance
        self.fullMap = fm
        self.runFilter = 0  # don't run the filter until you've moved enough
        self.runFilterLock = threading.Lock()
        self.poseAverage = pose.Pose(0, 0, 0)

    # displayPoses(): draws the current poseSet to rviz
    # may exhibit odd behavior while the filter is updating poses; see pose.py
    def displayPoses(self):
        self.poseSet.display()
        self.poseSet.displayOne(self.poseAverage, displayColor = [0, 0, 1])

    # receiveOdom(): updates the particle filter with the newest odometry reading
    # if the new reading is different enough, run the filter
    # parameters:
    #   odom -- the new odometry reading, currently in [[x, y], angle] format
    def receiveOdom(self, odom):
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
            self.predictionStep(motion)
            self.updatePoseAverage()
            self.displayPoses() # poses have changed, so draw the new ones

    def updatePoseAverage(self):
        self.poseAverage = pose.Pose(0,0,0)
        totalWeight = 0
        for p in self.poseSet.poses:
            self.poseAverage.x += p.x
            self.poseAverage.y += p.y
            self.poseAverage.theta += p.theta
            totalWeight += p.weight
        denom = 1.0/totalWeight
        self.poseAverage.x *= denom
        self.poseAverage.y *= denom
        self.poseAverage.theta *= denom
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
        if self.fullMap:    # if we have a valid map, cull the poses that are in illegal positions
            self.poseSet.poses = filter(lambda p: self.fullMap.inBounds(p), self.poseSet.poses)
