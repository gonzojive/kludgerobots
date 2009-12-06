import math
import statutil
import util
import vector

# class Pose
#   contains the robot pose info (x position, y position, theta) and a weight
class Pose:
    def __init__(self, x, y, theta, weight = 1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    def clonePose(self):
        return Pose(self.x, self.y, self.theta, self.weight)

    # the pose itself is in the map frame, we just need to undo the
    # rotation and apply the translation
    def inMapFrame(self, pt):
        # first rotate the pt about the z axis by the given angle
        ptRotated = vector.vector_rotate_2d(pt, 1.0 * self.theta)
        return vector.vector_add(ptRotated, [ self.x, self.y])

    def toStr(self):
        return "(%0.2f, %0.2f) at %0.2f degrees - weight = %0.2f" % (self.x, self.y, util.r2d(self.theta), self.weight)
            

# class PoseSet
#   contains the list of poses
class PoseSet:
    # __init__()
    # parameters:
    #   numPoses    --  the number of poses to track
    def __init__(self, numPoses):
        self.totalNumPoses = numPoses
        self.poses = []

    def avgWeight(self):
        length = len(self.poses)
        if length > 0.0:
            total = sum([float(o.weight) for o in self.poses])
            return total / float(length)
        else:
            return 0.0;
            

    def printPoses(self):
        print "Pose list (%d poses):" % len(self.poses)
        for p in self.poses:
            print p.toStr()
                

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
    def generateUniformPoses(self, xEnds, yEnds, thetaEnds, totalNumPoses):
        for i in xrange(0, totalNumPoses):
            [x, y, theta] = statutil.randomMultivariateUniform([xEnds, yEnds, thetaEnds])
            yield Pose(x, y, theta)

    def generateGaussianPoses(self, xParams, yParams, thetaParams, totalNumPoses):
        for i in xrange(0, totalNumPoses):
            [x, y, theta] = statutil.randomMultivariateGaussian([xParams, yParams, thetaParams])
            yield Pose(x, y, theta)
