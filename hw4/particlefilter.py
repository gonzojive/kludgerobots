import pose
import util
import statutil
import math
import mapmodel
import laser
import random
import cProfile
import vector

# Stuff for profiling, might as well leave it in
context = None
def profiledParticleFilter():
    return context.mainLoop()

# Particle filter, specialized for the final project
class ParticleFilter:
    def __init__(self, mapModel, initialPose, laser):
        global context
        context = self
        self.mapModel = mapModel    # already initialized by main
        self.poseSet = pose.PoseSet(5000)   # Number of poses to try
        # Kurt says position is within 1m and about 5 degrees of accurate
        bounds = [1, 1, util.d2r(5)]
        self.poseSet.initializeUniformStochastic( [initialPose.x-bounds[0], initialPose.x+bounds[0]], [initialPose.y-bounds[1], initialPose.y+bounds[1]], [initialPose.theta-bounds[2], initialPose.theta+bounds[2]] )
        self.laser = laser  # already initialized
        self.mapThresh = 0.2    # maximum distance to be part of the map
  

    # Initializes the profiler or just runs the main loop
    def run(self):
        import particlefilter
        #cProfile.run('particlefilter.profiledParticleFilter()', '/tmp/pfilterProfStats')
        return particlefilter.profiledParticleFilter()


    # The main particle filter loop
    # For each test pose, transforms the laser readings onto that pose, and
    # checks to see how accurate the fit is with the map
    # Returns the best guess for the pose, the laser readings that corresponded
    # with the map, and the laser readings that didn't fit with the map
    def mainLoop(self):
        self.cullIllegalPoses() # Probably won't be any, but it can't hurt
        self.updatePoseAverage()    # Not really necessary
        laserVectors = []
        # Only use points whose distance is < 12m
        for i in range(len(self.laser.points)):
            if self.laser.polarPoints[i][0] < self.laser.maxRadius:
                laserVectors.append(self.laser.points[i])
        bestPose = self.updateStep(laserVectors)    # do the probability calculations

        # It seems that the pose average gives a more consistent result than the
        # highest probability pose.
        returnPose = bestPose  # highest probability pose
        #returnPose = self.poseAverage   # average probability post
        [mapLasers, objectLasers] = self.classifyLasers(laserVectors, returnPose)
        return [returnPose, mapLasers, objectLasers]
        

    # Calculates the pose average based on all poses and their weights
    def updatePoseAverage(self):
        #rospy.loginfo("updating pose")
        thetaX = 0
        thetaY = 0
        totalWeight = 0
        newPose = pose.Pose(0, 0, 0)
        for p in self.poseSet.poses:
            newPose.x += p.x*p.weight
            newPose.y += p.y*p.weight
            thetaX += math.cos(p.theta)*p.weight
            thetaY += math.sin(p.theta)*p.weight
            totalWeight += p.weight
        if totalWeight < util.NUMERIC_TOL:
            newPose.x = 0.0
            newPose.y = 0.0
            newPose.setTheta(0.0)
            newPose.weight = 0.0
        else:
            denom = 1.0/totalWeight
            newPose.x *= denom
            newPose.y *= denom
            newPose.setTheta(math.atan2(thetaY*denom, thetaX*denom))
            newPose.weight = 1.0
        self.poseAverage = newPose


    # Calculates the probabilities of each pose, given the laser readings
    def updateStep(self, laserVectors):
        sparseLaserVectors = laserVectors[0:-1:4]   # every 4th reading
        maxWeight = -1  # to help find the highest probability pose
        for pose in self.poseSet.poses:
            pose.weight = self.pSensorReadingGivenPose(sparseLaserVectors, pose)
            if pose.weight > maxWeight:
                maxWeight = pose.weight
                best = pose
        self.normalizeWeights()
        self.resampleStep() # not sure if we should still do this
        self.updatePoseAverage()
        return best


    # Resample step: uses low-variance sampling to coalesce poses
    def resampleStep(self):
        #for p in self.poseSet.poses:
        weights = [p.weight for p in self.poseSet.poses]
        # we perform destructive acts on the poses, so clone them
        generatedPoses = statutil.lowVarianceSample2(self.poseSet.poses, weights, self.poseSet.totalNumPoses)
        sampledPoses = [p.clonePose() for p in generatedPoses]
        self.poseSet.poses = sampledPoses


    # pSensorReadingGivenPose()
    # given the lasers in cartesian coords and the pose, calculate the
    # probability of that pose
    def pSensorReadingGivenPose(self, laserBeamVectors, pose):
        # pseudocode:
        # for each laser beam in the laser scan, calculate the probabability
        pLaserBeamsGivenPose = [self.pLaserBeamGivenPose(vLaserBeam, pose) for vLaserBeam in laserBeamVectors]
        sumLogProbabilities = 0.0
        for p in pLaserBeamsGivenPose:
            sumLogProbabilities += math.log(p)

        result = math.exp(sumLogProbabilities)
        return result


    # classifyLasers()
    # Given the lasers in cartesian coords and the best guess for a pose, determine
    # which laser readings are close enough to be considered part of the map
    def classifyLasers(self, laserVecs, pose):
        globalLaserVecs = [pose.inMapFrame(l) for l in laserVecs]
        dists = [self.mapModel.distanceFromObstacleAtPoint(l) for l in globalLaserVecs]
        mapLasers = []
        objectLasers = []
        #print "Using distance cutoff = %0.2f to classify lasers" % (self.mapThresh)
        for i in range(len(laserVecs)):
            if dists[i] >= self.mapThresh:   # doesn't fit in the map
                objectLasers.append(laserVecs[i])
            else:   # close enough to the map
                mapLasers.append(laserVecs[i])
        #print "%d points match the map, %d points don't fit" % (len(mapLasers), len(objectLasers))
        return [mapLasers, objectLasers]


    def pLaserBeamGivenPose(self, vLaserBeam, pose):
        vLaserBeamInMapFrame = pose.inMapFrame(vLaserBeam)
        d = self.mapModel.distanceFromObstacleAtPoint(vLaserBeamInMapFrame)
        if d > self.mapThresh and self.mapModel.pointInBounds(vLaserBeamInMapFrame):
            return 1
        stdDev = 0.2
        pGauss = statutil.gaussianProbability(0, stdDev, d)
        pUniform = 1.0/self.laser.maxRadius
        weightGauss = .9
        weightUniform = .1
        prob = weightGauss*pGauss + weightUniform*pUniform
        return prob


    # cullIllegalPoses()
    # gets rid of all poses outside the map boundaries
    def cullIllegalPoses(self):
        self.poseSet.poses = filter(lambda p: self.mapModel.inBounds(p), self.poseSet.poses)
        

    def normalizeWeights(self):
        s = sum([p.weight for p in self.poseSet.poses])
        for pose in self.poseSet.poses:
            pose.weight = pose.weight / s
