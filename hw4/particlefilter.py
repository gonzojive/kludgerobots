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
        self.poseSet = pose.PoseSet()
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
        laserScan = [[laserScan[i], i] for i in xrange(0,len(laserScan))]
        laserVectors = laser.laserScanToVectors(laserScan)
        self.updateStep(laserVectors)
        self.updatePoseAverage()

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

    def mapFrameToLaserFrame(self, pt):
        (origin, theta) = ([self.poseAverage.x, self.poseAverage.y], self.poseAverage.theta)
        vToPtGlobal = vector_minus(pt, origin)
        rotated_vToGoal = vector_rotate_2d( vToPtGlobal, -1.0 * theta)
        return rotated_vToGoal

    
    def updateStep(self, laserVectors):
        # pseudocode:
        # for all poses in particle filter:
        #    transform the laser scans into that pose frame
        #    calculate P(sensor reading | pose)
        #       - sensor reading may consist of many laser beam readings which are multiplied together
        #    P( sensor reading | pose) becomes the new weight for the particle
        #    maybe normalize the weights
        avgWeight = self.poseSet.avgWeight()
        # this is not truly the weight per beam (that would be avgWeight ** float(self.numBeamVectors), but
        # it should sort of maybe in some cases correct for differing number of beams per cast.  And it works!
        avgWeightPerBeam = avgWeight * float(self.numBeamVectors)
        varPerBeamWeight = statutil.variance( [o.weight * float(self.numBeamVectors) for o in self.poseSet.poses] )

        poseNum = 0
        sparseLaserVectors = laserVectors[0:-1:8]   # every 8th reading
        for pose in self.poseSet.poses:
            pSensorReadingGivenPose = self.pSensorReadingGivenPose(sparseLaserVectors, pose)
            pose.weight = pSensorReadingGivenPose
            poseNum += 1
        self.normalizeWeights()
        self.resampleStep()
            

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
