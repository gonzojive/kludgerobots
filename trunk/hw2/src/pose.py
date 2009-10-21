#!/usr/bin/env python
#import roslib; roslib.load_manifest('hw2')
#import rospy
#import tf
import math
import statutil

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
    # __init__
    # parameter(): numPoses = the number of poses, either a list or a total value
    def __init__(self, viz, numPoses = [20, 20, 10]):    # default to 20 in x and y, 10 in theta
        if not isinstance(numPoses, list):  # if the argument is not a list (it's a number)
            posesPerDim = int(numPoses ** (1.0/3.0) + 0.5)  # nearest perfect cube
            self.numPoses = [posesPerDim, posesPerDim, posesPerDim] # same poses in each dim
        else:   # otherwise, it's a list, just use the values directly
            self.numPoses = numPoses
        self.poses = []
        self.viz = viz  # the Visualizer object that draws to rviz

    # drawArrows(): send each pose out to rviz as an arrow
    def drawArrows(self):
        for p in self.poses:
            self.viz.vizArrow([p.x, p.y], p.theta, length = p.weight)

    # prediction(): update each pose given old and new odometry readings
    # parameters:
    #   odomInitial --  an [x, y, angle] list of the previous odometry reading
    #   odomFinal   --  an [x, y, angle] list of the most recent odometry reading
    #   Note: Should we use Pose instead of a list? Not sure what format we want odometry in
    def prediction(self, odomInitial, odomFinal):
        [theta1, delta, theta2] = odomToMotionModel(odomInitial, odomFinal)    # convert from odom to 2 angles and a distance
        for p in self.poses:
            # calculate the error in each movement
            # once we get the gaussian function, use this here for each variable
            errTheta1 = 0
            errDelta = 0
            errTheta2 = 0
            [dx, dy, dtheta] = motionModelToOdom([theta1+errTheta1, delta+errDelta, theta2+errTheta2])
            p.x += dx
            p.y += dy
            p.theta += dtheta
            # update the weights based on the gaussian results?

    # initializeUniform(): sets a uniform distribution of poses in a given range
    # parameters:
    #   xEnds   --  an [xmin, xmax] list
    #   yEnds   --  a [ymin, ymax] list
    #   thetaEnds   --  a [thetamin, thetamax] list, defaults to [0, 2pi]
    # note: as it's written, the distribution includes the start pose but not the end pose
    # this is to avoid double-counting 0 and 2pi
    def initializeUniform(self, xEnds, yEnds, thetaEnds = [0,2*math.pi]):
        xInterval = float(xEnds[1] - xEnds[0]) / self.numPoses[0]
        yInterval = float(yEnds[1] - yEnds[0]) / self.numPoses[1]
        thetaInterval = float(thetaEnds[1] - thetaEnds[0]) / self.numPoses[2]
        x = xEnds[0]
        for i in range(self.numPoses[0]):
            y = yEnds[0]
            for j in range(self.numPoses[1]):
                theta = thetaEnds[0]
                for k in range(self.numPoses[2]):
                    self.poses.append(Pose(x, y, theta))
                    theta += thetaInterval
                y += yInterval
            x += xInterval
        self.normalizeWeights()


    # Initialize the points to a stachastic uniform distribution--randomly choosing points
    # between given values for x,y, theta
    # eachof xParams, yParams, and thetaParams are (min, max) pairs
    #  addes a bunch of poses to the poseSet and normalizes their weights
    def initializeUniformStochastic(self, xParams, yParams, thetaParams, totalNumPoses):
        newPoses = [pose for pose in self.generateUniformPoses(xParams, yParams, thetaParams, totalNumPoses)]
        self.poses += newPoses
        self.normalizeWeights()

    # initializeGaussian(): sets a uniform distribution of poses in a given range
    # parameters:
    #   xParams -- [mean, standard devition] for the x variable
    #   yParams -- [mean, standard devition] for the y variable
    #   thetaParams -- [mean, standard devition] for the theta variable
    # note: as it's written, the distribution includes the start pose but not the end pose
    # this is to avoid double-counting 0 and 2pi
    def initializeGaussian(self, xParams, yParams, thetaParams, totalNumPoses):
        newPoses = [pose for pose in self.generateGaussianPoses(xParams, yParams, thetaParams, totalNumPoses)]
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

# odomToMotionModel(): converts initial and final odom readings into our motion model variables
# parameters:
#   odomInitial --  [x, y, angle] list of the initial odometry
#   odomFinal   --  [x, y, angle] list of the final odometry
def odomToMotionModel(odomInitial, odomFinal):
    [dx, dy, da] = [odomFinal[0]-odomInitial[0], odomFinal[1]-odomInitial[1], odomFinal[2]-odomInitial[2]]    # calculate (final - initial)
    theta1 = math.atan2(dy, dx) - odomInitial[2]  # initial turn
    delta = math.sqrt(dx*dx + dy*dy)    # distance traveled
    theta2 = da - theta1   # final turn
    return [theta1, delta, theta2]

def motionModelToOdom(model):
    dx = model[1] * math.cos(model[0])
    dy = model[1] * math.sin(model[1])
    da = model[0] + model[2]
    return [dx, dy, da]
