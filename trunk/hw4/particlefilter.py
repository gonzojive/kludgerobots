import pose
import util
import statutil
import math
import mapmodel
import laser
import random
import cProfile
import vector


context = None

def profiledParticleFilter():
    context.mainLoop()

class ParticleFilter:
    def __init__(self, mapModel, initialPose, laser):
        global context
        context = self
        self.mapModel = mapModel
        self.poseSet = pose.PoseSet(5000)
        self.poseSet.initializeUniformStochastic( [initialPose.x-0.5, initialPose.y+0.5], [initialPose.y-0.5, initialPose.y+0.5], [initialPose.theta-0.0436, initialPose.theta+0.0436] ) # 1m x 1m, 5 degrees
        self.laser = laser
        self.numBeamVectors = 10
  

    def run(self):
        import particleFilter
        #cProfile.run('particleFilter.profiledParticleFilter()', '/tmp/pfilterProfStats')
        particleFilter.profiledParticleFilter()

    def mainLoop(self):
        self.cullIllegalPoses()
        self.updatePoseAverage()
        laserScan = [[laserScan[i], i] for i in xrange(0,len(self.laser.ranges))]
        laserVectors = laser.laserScanToVectors(laserScan)
        bestPose = self.updateStep(laserVectors)
        # Need to do some testing to see whether the best pose or
        # the average pose gives a better result
        return bestPose
        #return self.poseAverage
        

    def updatePoseAverage(self):
        #rospy.loginfo("updating pose")
        thetaX = 0
        thetaY = 0
        totalWeight = 0
        newPose = pose.Pose(0, 0, 0)
        for p in self.poseSet.poses:
            newPose.x += p.x*p.weight
            newPose.y += p.y*p.weight
            thetaX += math.cos(p.theta)
            thetaY += math.sin(p.theta)
            totalWeight += p.weight
        if totalWeight < util.NUMERIC_TOL:
            newPose.x = 0.0
            newPose.y = 0.0
            newPose.theta = 0.0
            newPose.weight = 0.0
        else:
            denom = 1.0/totalWeight
            newPose.x *= denom
            newPose.y *= denom
            newPose.theta = math.atan2(thetaY*denom, thetaX*denom)
            newPose.weight = 1.0
        self.poseAverage = newPose


    def updateStep(self, laserVectors):
        sparseLaserVectors = laserVectors[0:-1:4]   # every 4th reading
        best = pose.Pose()
        maxWeight = -1
        for pose in self.poseSet.poses:
            pose.weight = self.pSensorReadingGivenPose(sparseLaserVectors, pose)
            if pose.weight > maxWeight:
                maxWeight = pose.weight
                best = pose
        self.normalizeWeights()
        self.resampleStep()
        self.updatePoseAverage()


    def resampleStep(self):
        #for p in self.poseSet.poses:
        weights = [p.weight for p in self.poseSet.poses]
        # we perform destructive acts on the poses, so clone them
        generatedPoses = statutil.lowVarianceSample2(self.poseSet.poses, weights, self.numDesiredPoses)
        sampledPoses = [p.clonePose() for p in generatedPoses]
        self.poseSet.poses = sampledPoses


    def pSensorReadingGivenPose(self, laserBeamVectors, pose):
        # pseudocode:
        # for each laser beam in the laser scan, calculate the probabability
        self.numBeamVectors = len(laserBeamVectors)
        pLaserBeamsGivenPose = [self.pLaserBeamGivenPose(vLaserBeam, pose) for vLaserBeam in laserBeamVectors]
        sumLogProbabilities = 0.0
        for p in pLaserBeamsGivenPose:
            sumLogProbabilities += math.log(p)

        result = math.exp(sumLogProbabilities)
        #rospy.loginfo("sumLogProbabilities = %f,  p = %f", sumLogProbabilities, result)
        #result = sumLogProbabilities
        return result


    def pLaserBeamGivenPose(self, vLaserBeam, pose):
        vLaserBeamInMapFrame = pose.inMapFrame(vLaserBeam)
        d = self.mapModel.distanceFromObstacleAtPoint(vLaserBeamInMapFrame)
        stdDev = 1.34
        pGauss = statutil.gaussianProbability(0, stdDev, d)
        pUniform = 1.0/12.0
        weightGauss = .2
        weightUniform = .8
        return weightGauss*pGauss + weightUniform*pUniform


    def cullIllegalPoses(self):
        self.poseSet.poses = filter(lambda p: self.mapModel.inBounds(p), self.poseSet.poses)
        

    def normalizeWeights(self):
        s = sum([p.weight for p in self.poseSet.poses])
        for pose in self.poseSet.poses:
            pose.weight = pose.weight / s
