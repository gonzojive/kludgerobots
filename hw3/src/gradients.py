import rospy
import mapmodel
import math
import imageutil
import util
import vector
from vector import *
import viz
from numpy import allclose
import copy
import pose



ROBOT_RADIUS = 0.15
DISTANCE_CHANGE_POINT = 1.2
MAX_OBSTACLE_DISTANCE = 2.0
MAX_INTRINSIC = 10000.0
START_INTRINSIC = 100.0
MID_INTRINSIC = 10.0
END_INTRINSIC = 1.0
STEP_SIZE = .1

COST_THRESH_INITIAL = 5.0
COST_THRESH_INCREMENT = 5.0

NEW_OBSTACLE_THRESH = 5.0

#MapPoint is a structure to hold the gradient field values for all points in the map. gradient is the direction of the gradient at each (x,y) point. intrinsicVal is the distance to the nearest obstacle and goalVal is the distance to the goal
class MapPoint:
    def __init__(self, xIndex, yIndex, xPos, yPos):
        self.xInd = xIndex
        self.yInd = yIndex
        self.x = xPos
        self.y = yPos
        self.gradient = None
        self.intrinsicVal = 1
        self.cost = None
        



#GradientField is a class which creates and updates the gradient field for the map. It is generalized to perform global and local gradient updates
class GradientField:
    def __init__(self, cellSpacing, distanceMap, initialPose):
        # constants for calculating the intrinsic (obstacle distance) cost
        self.shallowSlope = (MID_INTRINSIC - START_INTRINSIC) / (DISTANCE_CHANGE_POINT - ROBOT_RADIUS)
        self.steepSlope = (END_INTRINSIC - MID_INTRINSIC) / (MAX_OBSTACLE_DISTANCE - DISTANCE_CHANGE_POINT)
        # data members
        self.spacing = cellSpacing
        self.gradientMap =  []
        self.stepSize = STEP_SIZE
        self.goals = []
        self.initializationDone = False
        self.localField = None
        self.globalField = None
        self.localObstacles = None
        if distanceMap:
            self.mapWidth = int(distanceMap.fWidth)
            self.mapHeight = int(distanceMap.fHeight)
            self.gridWidth = int(self.mapWidth / cellSpacing)
            self.gridHeight = int(self.mapHeight / cellSpacing)
            self.initializeGradientMap(distanceMap)
            self.startPosition = initialPose


    def setGlobal(self, g):
        self.globalField = g

    def setLocal(self, l):
        self.localField = l

    def initLocalFromGlobal(self):
        self.mapWidth = self.globalField.mapWidth
        self.mapHeight = self.globalField.mapHeight
        self.gridWidth = self.globalField.gridWidth
        self.gridHeight = self.globalField.gridHeight
        self.gradientMap = copy.deepcopy(self.globalField.gradientMap)
        self.goals = self.globalField.goals
        self.localObstacles = [[0]*self.gridHeight]*self.gridWidth
        self.localGoals = []


    def newLaserReadingOld(self, laserPoints):
        minPoint = [10000, 10000]
        maxPoint = [-1, -1]
        obsList = []
        self.localGoals = []
        for p in laserPoints:
            cell = self.cellNearestXY(p[0], p[1])
            if cell.xInd < minPoint[0]:
                minPoint[0] = cell.xInd
            if cell.xInd > maxPoint[0]:
                maxPoint[0] = cell.xInd
            if cell.yInd < minPoint[1]:
                minPoint[1] = cell.yInd
            if cell.yInd > maxPoint[1]:
                maxPoint[1] = cell.yInd
            if cell.intrinsicVal < NEW_OBSTACLE_THRESH and cell not in obsList:
                rospy.loginfo("Found new obstacle at cell (%0.2f, %0.2f)", cell.x, cell.y)
                obsList.append(cell)
                self.localObstacles[cell.xInd][cell.yInd] += 3
            else:
                if cell not in self.localGoals:
                    self.localGoals.append(cell)
        #rospy.loginfo("Range: (%d, %d) to (%d, %d)", minPoint[0], minPoint[1], maxPoint[0], maxPoint[1])

        for i in xrange(minPoint[0], maxPoint[0]):
            for j in xrange(minPoint[1], maxPoint[1]):
                self.gradientMap[i][j].cost = None
                self.gradientMap[i][j].gradient = None
                self.localObstacles[i][j] -= 2
                if self.localObstacles[i][j] > 0:
                    for x in xrange(i-4, i+5):
                        for y in xrange(j-4, j+5):
                            intCost = self.intrinsicFunc(abs((i-x)*self.spacing)+abs((j-y)*self.spacing))
                            if intCost > self.gradientMap[x][y].intrinsicVal:
                                self.gradientMap[x][y].intrinsicVal = intCost
        self.activeList = []
        for g in self.localGoals:
            self.gradientMap[g.xInd][g.yInd].cost = self.globalField.gradientMap[g.xInd][g.yInd].cost
            self.activeList.append(g)
        for g in self.goals:
            cell = self.cellNearestXY(g.x, g.y)
            cell.cost = 0
            self.activeList.append(cell)

    def newLaserReading(self, laserPoints):
        obsList = []
        rospy.loginfo("Started local computation")
        for p in laserPoints:
            cell = self.cellNearestXY(p[0], p[1])
            if cell.intrinsicVal < NEW_OBSTACLE_THRESH and cell not in obsList:
                rospy.loginfo("Found new obstacle at cell (%0.2f, %0.2f)", cell.x, cell.y)
                obsList.append(cell)
        #rospy.loginfo("Range: (%d, %d) to (%d, %d)", minPoint[0], minPoint[1], maxPoint[0], maxPoint[1])

        updateRange = 6
        updateList = []
        for o in obsList:
            for i in xrange(-updateRange,updateRange+1):
                x = o.xInd + i
                for y in xrange(o.yInd+abs(i)-updateRange, o.yInd+1+updateRange-abs(i)):
                    #intCost = self.intrinsicFunc((abs(o.xInd-x)+abs(o.yInd-y))/self.spacing)
                    intCost = 10 - (abs(o.xInd-x)+abs(o.yInd-y))
                    cell = self.gradientMap[x][y]
                    if intCost > cell.intrinsicVal:
                        #rospy.loginfo("Intrinsic cost was: %0.2f, now %0.2f", cell.intrinsicVal, intCost)
                        cell.intrinsicVal = intCost
        self.initializationDone = True

    def updatePath(self, position):
        self.startCell = self.cellNearestXY(position.x, position.y)
        for row in self.gradientMap:
            for point in row:
                point.cost = None
                point.gradient = None
        self.costThresh = COST_THRESH_INITIAL
        self.activeList = []
        self.highCostList = []
        for g in self.goals:
            cell = self.gradientMap[g.xInd][g.yInd]
            cell.cost = 0
            self.activeList.append(cell)
        while not self.foundStartPosition():
            self.calculateCosts(10)
        self.calculateGradients()

    def setStartPosition(self, x, y):
        self.startPosition = pose.Pose(x, y, 0)


    def setGoals(self, goals):
        if not self.startPosition:
            rospy.loginfo("Can't set goals without a valid start position")
            return
        for row in self.gradientMap:
            for point in row:
                point.cost = None
                point.gradient = None
        self.activeList = []
        self.goals = []
        for g in goals:
            rospy.loginfo("goal is (%0.2f,%0.2f)",g.x,g.y)
            cell = self.cellNearestXY(g.x, g.y)
            cell.cost = 0
            self.activeList.append(cell)
            self.goals.append(cell)
        self.startCell = self.cellNearestXY(self.startPosition.x, self.startPosition.y)
        self.costThresh = COST_THRESH_INITIAL
        self.highCostList = []

    def foundStartPosition(self):
        return self.startCell.cost != None
 
    def cellNearestXY(self, x, y):
        xIndex = int( float(x)/self.spacing + 0.5 )
        yIndex = int( float(y)/self.spacing + 0.5 )
        return self.gradientMap[xIndex][yIndex]

    def interpolateGradientAtXY(self, x, y):
        gridX = float(x)/self.spacing
        gridY = float(y)/self.spacing
        sw = self.gradientMap[int(gridX)][int(gridY)].gradient or [0, 0]
        se = self.gradientMap[int(gridX+1)][int(gridY)].gradient or [0, 0]
        nw = self.gradientMap[int(gridX)][int(gridY+1)].gradient or [0, 0]
        ne = self.gradientMap[int(gridX+1)][int(gridY+1)].gradient or [0, 0]
        alphaX = float(gridX - int(gridX))
        alphaY = float(gridY - int(gridY))
        north = [nw[0] + alphaX * (ne[0]-nw[0]), nw[1] + alphaX * (ne[1]-nw[1])]
        south = [sw[0] + alphaX * (se[0]-sw[0]), sw[1] + alphaX * (se[1]-sw[1])]
        gradient = [south[0] + alphaY * (north[0]-south[0]), south[1] + alphaY * (north[1]-south[1])]
        return gradient

    # initializeGradientMap()
    # initialize all the MapPoints using the pre-computed obstacle distance map
    def initializeGradientMap(self, distanceMap):   
        #now copy into the 2D Matrix
        rowNum =0
        curCol = []
        curX = 0.0
        tenPercent = float(self.gridWidth) / 10.0
        nextPrint = tenPercent
        curPercent = 10

        for i in xrange(0, self.gridWidth):
            curY = 0.0
            for j in xrange(0, self.gridHeight):
                mp = MapPoint(i, j, curX, curY)
                if distanceMap.pointInBounds([curX, curY]):
                    mp.intrinsicVal = self.intrinsicFunc(distanceMap.distanceFromObstacleAtPoint([curX, curY]))
                else:
                    mp.intrinsicVal = MAX_INTRINSIC
                curCol.append( mp )
                curY += self.spacing
            curX += self.spacing
            self.gradientMap.append(curCol)
            curCol = []
            if i > nextPrint:
                rospy.loginfo("%d%% complete", curPercent)
                curPercent += 10
                nextPrint += tenPercent
        rospy.loginfo("100% complete")


    def displayImageOfCosts(self, cutoff = 5.0):
        vals = []
        for c in self.gradientMap:
            for d in c:
                data = 0
                if not d.cost or d.cost > cutoff:
                    data = 0
                else:
                    data = cutoff - d.cost
                vals.append(data)
        imageutil.showImageRowCol(vals, self.gridHeight, self.gridWidth)

    def outputCosts(self, i):

        def allCells():
            for col in self.gradientMap:
                for cell in col:
                    yield cell
                    
        imageutil.outputMatrix([c.cost if c and c.cost else 0 for c in allCells()],
                               self.gridHeight, self.gridWidth,
                               filename="/tmp/costs-iter%i.mat" % i, i=i)

    def displayImageOfCosts2(self, cutoff = 25.0):
        vals = []
        def displayCostForCell(c):
            if not c.cost or c.cost > cutoff:
                data = cutoff
            else:
                data = c.cost
            return data

        def allCells():
            for col in self.gradientMap:
                for cell in col:
                    yield cell

        vals = [displayCostForCell(c) for c in allCells()]
        
        imageutil.showImageRowCol2(vals, self.gridHeight, self.gridWidth)
        imageutil.outputMatrix([c.cost if c and c.cost else 0 for c in allCells()],
                               self.gridHeight, self.gridWidth, "/tmp/costs.mat")


    def displayGradient(self, v):
        self.calculateGradients()
        def absVectorAtCell(cell):
            grad = vector_scale(cell.gradient, self.spacing)
            origin = [cell.x, cell.y]
            absVector = ( origin, vector_add(origin, grad))
            return absVector

        def allCells():
            for col in self.gradientMap:
                for cell in col:
                    if cell.gradient:
                        yield cell

        v.vizArrows([absVectorAtCell(cell) for cell in allCells()])
        self.initializationDone = True


    def displayImageOfIntrinsics(self, cutoff = 100.0):
        vals = []
        data = 0
        for c in self.gradientMap:
            for d in c:
                if d.intrinsicVal > cutoff:
                    data = 0
                else:
                    data = cutoff - d.intrinsicVal
                vals.append(data)
        imageutil.showImageRowCol(vals, self.gridHeight, self.gridWidth)

        

    # intrinsicFunc()
    # convert distances into intrinsic values, using the Konolidge paper as a model
    # very large value when distance < robot radius
    # gradually sloping down from there until a certain point
    # steep slope from there until the maximum distance
    # stays at end value (1.0) past there
    def intrinsicFunc(self, distance):
        if distance < ROBOT_RADIUS:
            return MAX_INTRINSIC
        elif distance < DISTANCE_CHANGE_POINT:
            return START_INTRINSIC + (self.shallowSlope * (distance - ROBOT_RADIUS))
        elif distance < MAX_OBSTACLE_DISTANCE:
            return MID_INTRINSIC + (self.steepSlope * (distance - DISTANCE_CHANGE_POINT))
        else:
            return END_INTRINSIC

    def sanityCheck(self):
        n = 0
        for xInd in xrange(0, self.gridWidth):
            for yInd in xrange(0, self.gridHeight):
                cell = self.gradientMap[xInd][yInd]
                assert(xInd == cell.xInd)
                assert(yInd == cell.yInd)
                n += 1
        rospy.loginfo("Sanity checked %i cells and they all passed", n)
    
    #function to find the 4 nearest neighbors
    def findNeighbors(self,point):
        neighbors = []
        x = point.xInd
        y = point.yInd
        if x > 0:
            neighbors.append(self.gradientMap[x-1][y])
        if x < self.gridWidth-1:
            neighbors.append(self.gradientMap[x+1][y])
        if y > 0:
            neighbors.append(self.gradientMap[x][y-1])
        if y < self.gridHeight-1:
            neighbors.append(self.gradientMap[x][y+1])
        return neighbors

    # finds a cell's N S E W neighbors
    def findCardinalNeighbors(self, n):
        N = self.cellIsValid(n.xInd, n.yInd+1)
        S = self.cellIsValid(n.xInd, n.yInd-1)
        E = self.cellIsValid(n.xInd+1, n.yInd)
        W = self.cellIsValid(n.xInd-1, n.yInd)
        return [ N, S, E, W]


    def cellIsValid(self, i, j):
        if i < 0 or i >= self.gridWidth or j < 0 or j >= self.gridHeight:
            return None
        cell = self.gradientMap[i][j]
        if cell.cost == None:
            return None
        return cell

    def calculateOneGradient(self, cell):
        N = self.cellIsValid(cell.xInd, cell.yInd+1)
        S = self.cellIsValid(cell.xInd, cell.yInd-1)
        E = self.cellIsValid(cell.xInd+1, cell.yInd)
        W = self.cellIsValid(cell.xInd-1, cell.yInd)
        grad = [0, 0]
        # Testing - remove before final
        if E == None and W == None:
            grad[0] = 0
        elif E == None:
            grad[0] = W.cost - cell.cost
        elif W == None:
            grad[0] = cell.cost - E.cost
        else:
            grad[0] = W.cost - E.cost
        if N == None and S == None:
            grad[1] = 0
        elif N == None:
            grad[1] = S.cost - cell.cost
        elif S == None:
            grad[1] = cell.cost - N.cost
        else:
            grad[1] = S.cost - N.cost
        if (vector_length_squared(grad) > .0001):
            grad = vector_normalize(grad)
        cell.gradient = grad


    def calculateGradients(self):
        for col in self.gradientMap:
            for cell in col:
                if not cell.cost:
                    continue
                self.calculateOneGradient(cell)

    # given north south east and west cells (that may be nil), 
    def wavefrontUpdateValue(self, N, S, E, W, intrinsicCost, cell=None):
        leastCostCell = None
        for cell in [ N, S, E, W ]:
            
            if cell and ((not leastCostCell) or cell.cost < leastCostCell.cost):
                leastCostCell = cell

        adjCells = (E, W) if (leastCostCell == N or leastCostCell == S) else (N, S)
        adjCells = filter(lambda x:x, adjCells)

        minCost = leastCostCell.cost
        adjCost = min([c.cost for c in adjCells]) if len(adjCells) > 0 else None
        
        def costFromCostsAtAdjCardinalDirections(Ta, Tc):
            assert(intrinsicCost >= .99)
            if Tc == None:
                #return Ta + intrinsicCost
                Tc = Ta + 100000.0
            assert(Ta <= Tc)
            # solve the quadratic  (T - Ta)^2 + (T-Tc)^2 = h^2 Fij^2
            # We assume that Fij = 0 because we do not really understand it
            # and this becomes the quadratic
            # (2) T^2  +  -2(Ta + Tc) T + (Ta^2 - Tc^2) - h^2 = 0
            h = 1.0 #self.spacing #0.0 #Fij * self.spacing
            #Fij = intrinsicCost
            Fij = 0.0 + intrinsicCost
            a = 2.0
            b = -2.0 * (Ta + Tc)
            c = Ta*Ta + Tc*Tc - (h*h * Fij * Fij)
            # solved by the quadratic formula
            roots = util.quadraticRoots(a, b, c)
            # only accept answers that are farther away than the minimum neighbor cost 
            roots = filter(lambda x: x.__class__ != complex and x > Tc, roots)
            #rospy.loginfo("roots for Ta = %f, Tc = %f; intrinsic = %f. a=%f b =%f c=%f: %s" % (Ta, Tc, intrinsicCost, a, b, c, roots))
            if len(roots) == 0:
                #rospy.loginfo("No real roots when Ta = %f, Tc = %f. a=%f b =%f c=%f, %s" % (Ta, Tc, a, b, c, util.quadraticRoots(a, b, c)))
                # degenerate case
                return math.sqrt(h * h * Fij * Fij) + Ta
            val = min(roots)
            assert(val >= 0)
            return val
        result = costFromCostsAtAdjCardinalDirections(minCost, adjCost)

        #rospy.loginfo("Min cost neighbors: %s, %s | our cost: %f" % (minCost, adjCost, result))
        return result

    # calculateCosts()
    # assumes the active list has been set, and goal costs have been set to 0
    #uses LPN algorithm, as in paper
    def calculateCosts(self, iterations = 70,goalIndex = 0, display=False):
        # start from the goals on the active list, and propagate the values from there
        temp =[]
        for i in range(iterations):
            if len(self.activeList) == 0:
                if len(self.highCostList) == 0:
                    rospy.loginfo("Both active lists are empty - no costs to calculate")
                    return
                #else:
                    #rospy.loginfo("Using the high cost list with threshold for high cost %f", self.costThresh)
                self.activeList = self.highCostList
                self.highCostList = []
                self.costThresh += COST_THRESH_INCREMENT
            while self.activeList:
                #currentPoint is the point whose value we know, and
                #which will be used to propagate values
                currentPoint = self.activeList.pop()
                if display:
                    rospy.loginfo("Active point: (%d, %d) in grid", currentPoint.xInd, currentPoint.yInd)
                
                #calculate the neighbors of the point and update them
                neighbors = self.findNeighbors(currentPoint)
                
                for n in neighbors:
                    # use the Sethian method to propagate the wavefront
                    [N, S, E, W] = self.findCardinalNeighbors(n)
                    if display:
                        rospy.loginfo("  Neighbor: (%d, %d) in grid", n.xInd, n.yInd)
                    for test in [N, S, E, W]:
                        if display:
                            if test:
                                rospy.loginfo("    (%d, %d) is valid", test.xInd, test.yInd)
                            else:
                                rospy.loginfo("    not valid")
                    newCost = self.wavefrontUpdateValue(N, S, E, W, n.intrinsicVal, n)
                    
                    if n.cost == None or newCost < n.cost:
                        n.cost = newCost
                        
                        if n.cost < self.costThresh:
                            temp.append(n)  # put onto active list
                        else:
                            self.highCostList.append(n)

                        if n == self.startCell:
                            rospy.loginfo("Updated starting point cost to %f", n.cost)
                #end for
            #end while

            # add the temp list to the active list, since the
            # values for all the entries in temp have been updated
            self.activeList.extend(temp)
            temp =[]
            #if (i+1)%10 == 0:
                #rospy.loginfo("iteration %d done",i+1)
         #end iterations
        rospy.loginfo("Cost calculation finished")
        if self.localField:
            self.localField.initLocalFromGlobal()
       

    def findPathGivenGradient(self, v, position = None):
        self.path =[]
        currPos = position or [self.startPosition.x, self.startPosition.y]
        goals = [[g.x, g.y] for g in self.goals]
        rospy.loginfo("Finding path to nearest goal from %0.2f,%0.2f", currPos[0], currPos[1])
        maxPath = 200
        while len(self.path) < maxPath and not util.closeToOne(currPos, goals):
            interpedGrad = self.interpolateGradientAtXY(currPos[0],currPos[1])
            #rospy.loginfo("path gradient: (%.2f, %.2f) of len %f", interpedGrad[0], interpedGrad[1], vector_length(interpedGrad))
            currPos = vector_add(currPos, vector_scale(interpedGrad, self.stepSize))
            self.path.append(currPos)
        #rospy.loginfo("Found goal. Path is:")
        v.vizConnectedPoints(self.path, color=[0,0,1])
        return self.path
