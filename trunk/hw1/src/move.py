import roslib;
import rospy
import tf
from math import *
from line import *
from util import *
from vector import *
from lineviz import *
from robotPosition import *

viz = None


class Trajectory:
    MAX_VEL = 0.9
    MAX_ANG_VEL = math.pi
    MAX_ACCEL = 0.5
    MAX_ANG_ACCEL = math.pi / 1.6
    SIMULATION_DT = .3
    SIMULATION_T = 4.0
    # constructs a velocity from provided robot controls (forward velocity and angular velocity)
    #
    # forward velocity is a scalar value in meters per second and is the velocity of the robot in
    #   the positive X direction
    #
    # angularVelocity is a scalar value in radians per second and it corresponds to the rotation
    #   of the robot about its center of rotation
    def __init__(self, forwardVelocity, angularVelocity, currentFwdVel, currentAngVel):
        self.fwdVel = forwardVelocity
        self.angVel = angularVelocity
        self.currentFwdVel = currentFwdVel
        self.currentAngVel = currentAngVel

    def vizTrajectory(self, name=None, color=None, z=.2):
        positions = self.simulate(Trajectory.SIMULATION_DT, Trajectory.SIMULATION_T)
        positions = [[0.0,0.0]] + positions
        #displayVectors = map(vector_minus, positions[1:], positions[: len(positions)-1])
        positions = map(lambda xy: (xy[0], xy[1], z), positions)
        rospy.loginfo("Visualizing Trajectory (%0.2f, %0.2f): %s", self.fwdVel, self.angVel, positions)
        viz.vizSegments(positions, name=name, color=color)

    def cost(self, simulationTime, occGrid, goal):
        positions = self.simulate(Trajectory.SIMULATION_DT, simulationTime)
        gridCellsSeen = {}
        def costAtPosition(xy):
            gridCell = occGrid.getGridValue(xy)
            if gridCell not in gridCellsSeen:
                gridCellsSeen[gridCell] = True
                obstCost = occGrid.obstacleCostAt(xy[0], xy[1])
                denom = float(len(gridCellsSeen))
                factor = 1.0 / (denom * denom)
                return factor * obstCost
            return 0

        # average trajectory object collision costs
        allObsCosts = map(costAtPosition, positions)
        ocost = sum(allObsCosts) #/ float(len(allObsCosts))

        # calculate distanceToGoal from endpoint of trajectory
        lastPosition = positions[len(positions) - 1]
        gcost = occGrid.distToGoal(lastPosition[0], lastPosition[1])

        # speed cost is inversely proportional to the speed we are going
        scost = -1.0 * vector_dot([self.fwdVel, 0], vector_normalize(goal))

        ocoef = 0.25
        gcoef = 4.0
        scoef = 2.4
        totalCost = scost * scoef + ocost * ocoef + gcost * gcoef

        rospy.loginfo("Trajectory (%0.2f, %0.2f) Cost: %s / speed: %0.2f / goal: %0.2f / obstac: %0.2f",
                      self.fwdVel, self.angVel, totalCost, scost * scoef, gcost * gcoef, ocost * ocoef)

        #rospy.loginfo("all obstacle costs: %s", map(lambda x : "%0.2f" % x, allObsCosts))
        #rospy.loginfo("all positions (%0.2f, %0.3f)  : %s", self.fwdVel, self.angVel, map(lambda x : "(%0.2f, %0.2f)" % (x[0], x[1]), positions))
        #rospy.loginfo("all cost at 0, 0: %s, bucket: %s", occGrid.obstacleCostAt(0.0, 0.0), occGrid.pointToBucketXY([0,0]))
        #

        return totalCost
            
    # given a time period T, calculates where the robot will be in [x,
    # y] coordinates relative to the initial position of the robot
    # with time steps DT.  Assumes that the angular velocity will be
    # constantly applied over this period of time given a returns
    def simulate(self, dt, simulationTime):
        # invariants
        maxDeltaFwdVel = Trajectory.MAX_ACCEL * dt
        maxDeltaAngVel = Trajectory.MAX_ANG_ACCEL * dt
        # change with each step
        theta = 0.0
        [x, y] = [0.0, 0.0]
        fwdVel = self.currentFwdVel
        angVel = self.currentAngVel
        t = dt
        positions = []
        while t < simulationTime + .0001:
            # set the change in angular and forward velocity and clamp it to max
            deltaVel = self.fwdVel - fwdVel
            deltaAngVel = self.angVel - angVel

            if deltaVel < -maxDeltaFwdVel:
                deltaVel = -maxDeltaFwdVel
            elif deltaVel > maxDeltaFwdVel:
                deltaVel = maxDeltaFwdVel
            
            if deltaAngVel < -maxDeltaAngVel:
                deltaAngVel = -maxDeltaAngVel
            elif deltaAngVel > maxDeltaAngVel:
                deltaAngVel = maxDeltaAngVel

            angVel += deltaAngVel
            fwdVel += deltaVel
            
            newTheta = theta + dt * angVel  # calculate theta after dt
            halfwayTheta = (newTheta + theta)/2.0  # find the halfway theta
            theta = newTheta
            
            x = x + fwdVel*cos(halfwayTheta)*dt  # estimate the x position after dt
            y = y + fwdVel*sin(halfwayTheta)*dt  # estimate the y position after dt
            t += dt
            positions = positions + [ [x, y] ]
              
        return positions

class MoveToGoal:
    def __init__(self, robotPosition, laserInterpreter):
        global viz
        self.xGoal = 4  # 4m
        self.thetaGoal = 180  # 180 degrees
        self.rp = robotPosition  # the RobotPosition
        self.li = laserInterpreter
        self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        self.stage = 1 # go through 4 stages: 1. go fwd 10 feet. 2. turn around. 3.
        self.viz = LocalMapVisualizer()
        self.occGrid = OccupancyGrid(self.li, self.viz)
        viz = self.viz
    def setGoal(self, goalXY):
        self.goal = goalXY
        self._goalSafe = goalXY
        self.paintGoal()

    def paintGoal(self):
        vOrigin = self.rp.origin()
        # figure out the vector to the goal
        vToGoal = self.vectorToGoal()

        self.viz.vizPoints([vToGoal])

    # returns the vector from the robot to the goal
    def vectorToGoal(self):
        # vOrigin and vToGoal are in global coordinates
        return self.rp.globalToLocal(self._goalSafe)

    def robotRadius(self):
        return .2 # 1.5 feet sounds not too bad for the width

    def maxLinearVelocity(self):
        return .5 # half a meter per second

    def publishNextVelocityAvoidingObstaclesOccGrid(self):
        
        #rotated_vForward = vector_rotate_2d( vForward, -1.0 * self.rp.theta())
        rospy.loginfo("theta in degrees: %0.2f", r2d(self.rp.theta()))
        vToGoal = self.vectorToGoal()
        #self.viz.vizSegment([0.0,0.0,0.0], [1.0,0.0,0.0], name="forward", color=[1.0, 1.0, 1.0])
        self.viz.vizSegment([0.0,0.0,0.0], vToGoal, name="vToGoal", color=[0.0,0.0,1.0])
        #self.viz.vizSegment([0,0,0], rotated_vForward, name="vForward")

        # update the occupancy grid
        self.occGrid.updateGrid(vToGoal)

        distanceToGoal = vector_length(vToGoal)
        newWVel = 0
        newXVel = -.1

        if distanceToGoal < .25:
            newXVel = 0.0
        else:
            # find the best trajectory
            bestTraj = self.findBestTrajectory()
            if bestTraj:
                bestTraj.vizTrajectory("BestTrajectory", color=(1.0, 0.0, 0.0), z=.3)
                
                newWVel = bestTraj.angVel
                newXVel = bestTraj.fwdVel

        self.velPublish.publish( Twist(Vector3(newXVel, 0, 0),Vector3(0, 0, newWVel)) )

    def findBestTrajectory(self):
        bestTraj = None
        bestCost = 1000000
        for traj in self.trajectoryGenerator():
            c = traj.cost(Trajectory.SIMULATION_T, self.occGrid, self.goal)
            rospy.loginfo("Trajectory (%0.2f, %0.2f) has cost %0.2f", traj.fwdVel, traj.angVel, c)
            if c < bestCost or not bestTraj:
                bestCost = c
                bestTraj = traj
        rospy.loginfo("Best Trajectory (%0.2f, %0.2f) with cost %0.2f", bestTraj.fwdVel, bestTraj.angVel, bestCost)
        return bestTraj
        
    def trajectoryGenerator(self):
        minVel = 0.1
        maxVel = 1.2
        minAngVel = -pi / 3.0
        maxAngVel =  pi / 3.0
        velSteps = 6
        angSteps = 8
        trajNum = 0
        # derived values
        velStep = (maxVel - minVel) / float(velSteps)
        angVelStep = (maxAngVel - minAngVel) / float(angSteps)
        for i in xrange(0, velSteps):
            fwdVel = float(i) * velStep + minVel
            for j in xrange(0, angSteps):
                trajNum += 1
                angVel = float(j) * angVelStep + minAngVel
                # TODO actually get current angular and fwd velocities
                traj = Trajectory(fwdVel, angVel, .2, 0)
                traj.vizTrajectory(name="traj %i" % trajNum)
                yield traj
        
    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self):
        if not self.goal:
            # by default just move forward
            self.velPublish.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))
            return
        else:
            #self.publishNextVelocityAvoidingObstacles()
            self.publishNextVelocityAvoidingObstaclesOccGrid()
            return

def mapFloatIntoDiscretizedBucket(f, minFloat, maxFloat, numBuckets):
    # f prefix float i discrete
    fSizeOfBucket = float(maxFloat - minFloat) / float(numBuckets)
    iBucket = int( float(f - minFloat) / fSizeOfBucket)
    if iBucket < 0:
        return 0
    elif iBucket >= numBuckets:
        return numBuckets - 1
    else:
        return iBucket

class OccupancyGridCell:
    def __init__(self):
        self.distanceToObstacle = 0.3
        self.collidesWithObstacle = None
        self.distanceToGoal = 10.0
        
class OccupancyGrid:
    def __init__(self, li, viz):
        self.li = li # laser interpreter
        self.minX = 0.0
        self.maxX = 2.5
        self.minY = -2.5
        self.maxY = 2.5
        #self.bucketsPerDimension = 20
        self.bucketsX = 17
        self.bucketsY = 14
        self.viz = viz
        self.grid = [OccupancyGridCell() for x in range(0, self.bucketsX * self.bucketsY)]
        self._bucketCenters = None
        self.goal = [0, 0]
        
    #givena  tuple containing floats, returns the value in the occupancy grid 
    def getGridValue(self, point):
        [x, y] = self.pointToBucketXY(point)
        return self.getBucketValue( x, y)
    
    def setGridValue(self, point, value):
        [x, y] = self.pointToBucketXY(point)
        self.setBucketValue(x, y, value)

    
    def setBucketValue(self, x, y, value):
        self.grid[x * self.bucketsY + y] = value
        
    def getBucketValue(self, x, y):
        return self.grid[x * self.bucketsY + y]

    def bucketCenters(self):
        if not self._bucketCenters:
            self._bucketCenters = [x for x in self.computeBucketCenters()]
        return self._bucketCenters

    def computeBucketCenters(self):
        stepX = self.calculateSpacing(True)
        stepY = self.calculateSpacing(False)
        for iX in range(0, self.bucketsX):
            fX = float(iX) * stepX + self.minX
            for iY in range(0, self.bucketsY):
                fY = float(iY) * stepY + self.minY
                yield [fX, fY, iX, iY]

    # returns integer [iX, iY] of the bucket that corresponds to the given floating point values                    
    def pointToBucketXY(self, pt):
        [x, y] = pt
        # if we are looking for -2 1 in a grid, we would return 
        xBucket = mapFloatIntoDiscretizedBucket(x, self.minX, self.maxX, self.bucketsX)
        yBucket = mapFloatIntoDiscretizedBucket(y, self.minY, self.maxY, self.bucketsY)
        return [xBucket, yBucket]

    # calculates the spacing in the given dimension (True for X , False for Y)
    def calculateSpacing(self, xdimp):
        if xdimp:
            return float(self.maxX - self.minX) / float(self.bucketsX)
        else:
            return float(self.maxY - self.minY) / float(self.bucketsY)

    # vizualizes the occupancy griz via rviz.  creates a line segment from the ground up
    # where the z is determined by the distance to the nearest obstacle
    def vizGrid(self):
        #rospy.loginfo("grid spacing => %0.2f", self.calculateSpacing())
        for [fX, fY, iX, iY] in self.bucketCenters():
            gridCell = self.getGridValue([fX, fY])
            #gridVal = gridCell.distanceToObstacle
            gridVal = self.obstacleCostAt(fX, fY)
            rospy.loginfo("viz grid [%0.2f, %0.2f] => %0.2f", fX, fY, gridCell.distanceToObstacle)
            self.viz.vizSegment([fX,fY,0.0], [fX,fY,gridVal * .2 ], name="forward %i %i" % (iX, iY), color=(0.0, .7, .2))

    def updateGridCollisions(self):
        # first update the collidsWithObstacle part of each gridcell using the laser readings
        for [fX, fY, iX, iY] in self.bucketCenters():
            vec = [fX, fY]
            # decide whether there is something in or before the
            # grid square by casting a ray in that direction
            cast_distance = self.li.castVector(vec)
            fY = -1.0 * fY
            vec = [fX, fY]
            distance_to_fXfY_squared = vector_length_squared(vec)
            # 0 weight for 
            gridCell = self.getGridValue(vec)
            obstacle_in_grid_squarep = cast_distance * cast_distance < distance_to_fXfY_squared
            #rospy.loginfo("i [%i, %i] f [%0.2f, %0.2f] cast to => %0.2f", iX, iY, fX, fY, cast_distance)
            if obstacle_in_grid_squarep:
                gridCell.distanceToObstacle = 0
                gridCell.collidesWithObstacle = True
            else:
                gridCell.distanceToObstacle = cast_distance - sqrt(distance_to_fXfY_squared)
                gridCell.collidesWithObstacle = False
            #self.setGridValue(vec, cast_distance - sqrt(distance_to_fXfY_squared))

    def updateGridDistances(self, goal):
        self.goal = goal
        # update distanceToObstacle for each cell
        for [fX, fY, iX, iY] in self.bucketCenters():
            dist = self.computeDistToNearestObstacle(fX, fY)
            gridCell = self.getGridValue([fX, fY])
            gridCell.distanceToObstacle = dist
            gridCell.distanceToGoal = vector_length(vector_minus(goal, [fX, fY]))

    # update the grid with the laser scan info
    def updateGrid(self, goal):
        # first update the collidsWithObstacle part of each gridcell using the laser readings
        self.updateGridCollisions()
        # then update the distanceToObstacle part of each gridCell using the collision data
        self.updateGridDistances(goal)
        #visualize the grid
        self.vizGrid()

    # distToNearestObstacle:
    #   An occupancy grid function - probably want this in the class
    #   Does an inefficient search for the nearest obstance in the grid
    def distToNearestObstacle(self, startX, startY):
        gridCell = self.getGridValue([startX, startY])
        return gridCell.distanceToObstacle

    def distToGoal(self, startX, startY):
        return vector_length(vector_minus([startX, startY], self.goal))
        gridCell = self.getGridValue([startX, startY])
        return gridCell.distanceToGoal

    def obstacleCostAt(self, x, y):
        obsDist = self.distToNearestObstacle(x, y)
        denom = obsDist + .001
        return 1.0 / (denom*denom)
        ocost = obsDist
        if obsDist < 0.2:  # too close to an obstacle -- invalidate this option
            ocost = 1000.0  # will make the cost very large if we hit an obstacle
        return ocost
    
    def computeDistToNearestObstacle(self, startX, startY):
        minDist = 4.0
        # simply loops over all the buckets and find the one closest to us that is an obstacle
        # OMG! O(n^4)!  Nonetheless, for a 12x12 grid that is only 20,000 iterations.  Why program
        # an efficient algorithm when this will work for our purposes?
        for [fX, fY, iX, iY] in self.bucketCenters():
            vec = [fX, fY]
            gridCell = self.getGridValue(vec)
            if gridCell.collidesWithObstacle:
                vToStart = vector_minus([startX, startY], vec)
                dist = vector_length(vToStart)  #abs(vec[0]-startX) + abs(vec[1]-startY)
                dist += -.343
                if dist < 0:
                    dist = 0
                if dist < minDist:
                    minDist = dist
        return minDist
