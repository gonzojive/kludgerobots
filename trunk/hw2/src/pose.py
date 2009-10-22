import math
import statutil
import motionModel
import rospy
import util


# class Pose
#   contains the robot pose info (x position, y position, theta) and a weight
class Pose:
    def __init__(self, x, y, theta, weight = 1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


# class PoseSet
#   contains the list of poses
class PoseSet:
    # __init__()
    # parameters:
    #   viz --  the visualizer object that draws to rviz
    #   err --  the error model to use
    #   numPoses    --  the number of poses to track
    #   odom    --  the initial odomety reading
    def __init__(self, viz, err = None, numPoses = 3000, odom = [[0, 0], 0]):
        self.totalNumPoses = numPoses
        self.poses = []
        self.viz = viz
        self.error = err or motionModel.ErrorModel()   # with no arguments, we should get 0 variance
        self.lastOdom = odom[0] + [odom[1]]  # convert from [[x, y], a] to [x, y, a]

    # drawArrows(): send each pose out to rviz as an arrow
    def display(self, type = "arrows"):
        idNumber = 0
        if type == "arrows":
            for p in self.poses:
                self.viz.vizArrow([p.x, p.y], p.theta, size = [0.2, 0.5, 0.5], idNum = idNumber)
                idNumber += 1

    def printPoses(self):
        rospy.loginfo("Pose list (%d poses):", len(self.poses))
        for p in self.poses:
            rospy.loginfo("  (%0.2f, %0.2f) at %0.2f degrees - weight = %0.2f", p.x, p.y, util.r2d(p.theta), p.weight)

    # predictionStep(): update each pose given old and new odometry readings
    # parameters:
    #   odomInitial --  an [x, y, angle] list of the previous odometry reading
    #   odomFinal   --  an [x, y, angle] list of the most recent odometry reading
    #   Note: Should we use Pose instead of a list? Not sure what format we want odometry in
    def predictionStep(self, newOdom):
        odomTriple = newOdom[0] + [newOdom[1]]  # convert from [[x, y], a] to [x, y, a]
        if max( [abs(x - y) for x, y in zip(self.lastOdom, odomTriple)] ) < 0.01:   # no real movement
            return
        # convert from odom to motion model (angle1, distance, angle2)
        motion = motionModel.odomToMotionModel(self.lastOdom, odomTriple)
        self.lastOdom = odomTriple  # save this new odometry
        for p in self.poses:
            predict = motionModel.Motion(0,0,0)
            # calculate the error in each movement, and make a new estimated motion
            # don't really want to take a square root every time here, can we just use the variance?
            predict.theta1 = statutil.randomGaussian(motion.theta1, math.sqrt(self.error.theta1Variance(motion)))
            predict.delta = statutil.randomGaussian(motion.delta, math.sqrt(self.error.deltaVariance(motion)))
            predict.theta2 = statutil.randomGaussian(motion.theta2, math.sqrt(self.error.theta2Variance(motion)))
            # convert back to odometry pose
            [dx, dy, dtheta] = motionModel.motionModelToOdom(predictMotion)
            p.x += dx
            p.y += dy
            p.theta += dtheta
            # do we update the weights based on the gaussian results?

    # initializeUniform(): sets a (deterministic) uniform distribution of poses in a given range
    # parameters:
    #   xEnds   --  an [xmin, xmax] list
    #   yEnds   --  a [ymin, ymax] list
    #   thetaEnds   --  a [thetamin, thetamax] list, defaults to [0, 2pi]
    # note: as it's written, the distribution includes the start pose but not the end pose
    # this is to avoid double-counting 0 and 2pi
    def initializeUniform(self, xEnds, yEnds, thetaEnds = [0,2*math.pi]):
        posesPerDim = int( self.totalNumPoses ** (1.0/3.0) + 0.5 )
        xInterval = float(xEnds[1] - xEnds[0]) / posesPerDim
        yInterval = float(yEnds[1] - yEnds[0]) / posesPerDim
        thetaInterval = float(thetaEnds[1] - thetaEnds[0]) / posesPerDim
        x = xEnds[0]
        for i in range(posesPerDim):
            y = yEnds[0]
            for j in range(posesPerDim):
                theta = thetaEnds[0]
                for k in range(posesPerDim):
                    self.poses += Pose(x, y, theta)
                    theta += thetaInterval
                y += yInterval
            x += xInterval
        self.normalizeWeights()


    # Initialize the points to a stachastic uniform distribution--randomly choosing points
    # between given values for x,y, theta
    # eachof xParams, yParams, and thetaParams are (min, max) pairs
    #  addes a bunch of poses to the poseSet and normalizes their weights
    def initializeUniformStochastic(self, xParams, yParams, thetaParams):
        newPoses = [pose for pose in self.generateUniformPoses(xParams, yParams, thetaParams, self.totalNumPoses)]
        self.poses += newPoses
        self.normalizeWeights()

    # initializeGaussian(): sets a uniform distribution of poses in a given range
    # parameters:
    #   xParams -- [mean, standard devition] for the x variable
    #   yParams -- [mean, standard devition] for the y variable
    #   thetaParams -- [mean, standard devition] for the theta variable
    # note: as it's written, the distribution includes the start pose but not the end pose
    # this is to avoid double-counting 0 and 2pi
    def initializeGaussian(self, xParams, yParams, thetaParams):
        newPoses = [pose for pose in self.generateGaussianPoses(xParams, yParams, thetaParams, self.totalNumPoses)]
        self.poses += newPoses
        self.normalizeWeights()

    # Ensures that the weights sum to 1
    def normalizeWeights(self):
        totalWeight = sum([ pose.weight for pose in self.poses])
        if totalWeight > 0:
            for pose in self.poses:
                pose.weight = pose.weight / totalWeight


    # initializeGaussian(): sets a uniform distribution of poses in a given range
    # parameters:
    #   xParams -- [mean, standard devition] for the x variable
    #   yParams -- [mean, standard devition] for the y variable
    #   thetaParams -- [mean, standard devition] for the theta variable
    # note: as it's written, the distribution includes the start pose but not the end pose
    # this is to avoid double-counting 0 and 2pi
    def generateUniformPoses(self, xEnds, yEnds, thetaEnds, totalNumPoses):
        for i in xrange(0, totalNumPoses):
            [x, y, theta] = statutil.randomMultivariateUniform([xEnds, yEnds, thetaEnds])
            yield Pose(x, y, theta)

    def generateGaussianPoses(self, xParams, yParams, thetaParams, totalNumPoses):
        for i in xrange(0, totalNumPoses):
            [x, y, theta] = statutil.randomMultivariateGaussian([xParams, yParams, thetaParams])
            yield Pose(x, y, theta)
