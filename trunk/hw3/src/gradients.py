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



ROBOT_RADIUS = 0.15
DISTANCE_CHANGE_POINT = 1.0
MAX_OBSTACLE_DISTANCE = 1.2
MAX_INTRINSIC = 10000.0
START_INTRINSIC = 100.0
MID_INTRINSIC = 50.0
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
        

class Goals:
    def __init__(self,x,y):
        self.x = x;
        self.y = y;
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

        self.initializationDone = None
        
        

        self.localField = None
        self.globalField = None
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


    def newLaserReading(self, laserPoints):
        for p in laserPoints:
            if self.cellNearestXY(p[0], p[1]).intrinsicVal < NEW_OBSTACLE_THRESH:
                rospy.loginfo("Found new obstacle at (%0.2f, %0.2f)", p[0], p[1])
        

    def setGoals(self, goals):
        if not self.startPosition:
            rospy.loginfo("Can't set goals without a valid start position")
            return
        for row in self.gradientMap:
            for point in row:
                point.cost = None
                point.gradient = None
        self.activeList = []
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
        self.initializationDone = 1


    def displayImageOfIntrinsics(self, cutoff = 100.0):
        vals = []
        for c in self.gradientMap:
            for d in c:
                data = 0
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


    def cellIsValid(self, i, j):
        if i < 0 or i >= self.gridWidth or j < 0 or j >= self.gridHeight:
            return None
        cell = self.gradientMap[i][j]
        if cell.cost == None:
            return None
        return cell


    def calculateGradients(self):
        for col in self.gradientMap:
            for cell in col:
                if not cell.cost:
                    continue
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

    # given north south east and west cells (that may be nil), 
    def wavefrontUpdateValue(self, N, S, E, W):
        leastCostCell = None
        for cell in [ N, S, E, W ]:
            
            if cell and ((not leastCostCell) or cell.cost < leastCostCell.cost):
                leastCostCell = cell

        adjCells = (E, W) if (leastCostCell == N or leastCostCell == S) else (N, S)
        adjCells = filter(lambda x:x, adjCells)

        minCost = leastCostCell.cost
        adjCost = min([c.cost for c in adjCells]) if len(adjCells) > 0 else None

        def costFromCostsAtAdjCardinalDirections(Ta, Tc):
            if not Tc:
                return Ta
            # solve the quadratic  (T - Ta)^2 + (T-Tc)^2 = h^2 Fij^2
            # We assume that Fij = 0 because we do not really understand it
            # and this becomes the quadratic
            # (2) T^2  +  -2(Ta + Tc) T + (Ta^2 - Tc^2) - h^2 = 0
            h = 0.0 #Fij * self.spacing
            a = 2.0
            b = -2.0 * (Ta + Tc)
            c = Ta*Ta - Tc*Tc - h*h
            # solved by the quadratic formula
            [root1, root2] = util.quadraticRoots(a, b, c)
            return min(root1, root2)
        return costFromCostsAtAdjCardinalDirections(minCost, adjCost)

    # calculateCosts()
    # assumes the active list has been set, and goal costs have been set to 0
    #uses LPN algorithm, as in paper
    def calculateCosts(self, iterations = 70):
        # start from the goals on the active list, and propagate the values from there
        temp =[]
        for i in range(iterations):
            if len(self.activeList) == 0:
                if len(self.highCostList) == 0:
                    rospy.loginfo("Both active lists are empty - no costs to calculate")
                    return
                self.activeList = self.highCostList
                self.highCostList = []
                self.costThresh += COST_THRESH_INCREMENT
            while self.activeList:
                currentPoint = self.activeList.pop() #currentPoint is the point whose value we know, and which will be used to propagate values
                
                #calculate the neighbors of the point
                neighbors = self.findNeighbors(currentPoint)
                
                #neighbors stores the neighbors of the point. 
                #Now we have to find the minimum neighbors along N-S and E-W for each of these neighbors
                for n in neighbors:
                    #find the minimum
                    N = self.cellIsValid(n.xInd, n.yInd+1)
                    S = self.cellIsValid(n.xInd, n.yInd-1)
                    E = self.cellIsValid(n.xInd+1, n.yInd)
                    W = self.cellIsValid(n.xInd-1, n.yInd)
                    NS = None
                    EW = None
                    minPoint = None
                    # North-South comparison
                    if N != None:
                        if S != None:
                            # Case 1: Both are valid - find the minimum
                            if N.cost < S.cost:
                                NS = N
                            else:
                                NS = S          
                        else:
                            # Case 2: N is valid, S is not - take N
                            NS = N
                    else:
                        if S != None:
                            # Case 3: S is valid, N is not - take S
                            NS = S
                       
                    # East-West comparison
                    if E != None:
                        if W != None:
                            # Case 1: Both are valid - find the minimum
                            if E.cost < W.cost:
                                EW = E
                            else:
                                EW = W         
                        else:
                            # Case 2: E is valid, W is not - take E
                            EW = E
                    else:
                        if W != None:
                            # Case 3: W is valid, E is not - take W
                            EW = W

                    # Calculate the correct theta, based on NS and EW
                    if NS != None:
                        if EW != None:
                            traj = [EW.cost, NS.cost]
                            if NS.cost < EW.cost:
                                minPoint = NS
                            else:
                                minPoint = EW
                        else:
                            traj = [1.0, 0.0]
                            minPoint = NS
                    else:
                        if EW != None:
                            traj = [0.0, 1.0]
                            minPoint = EW
                        else:
                            rospy.loginfo("Error! Grid [%d][%d] at [%0.2f, %0.2f] has NO valid neighbors", n.xInd, n.yInd, n.x, n.y)
                            continue

                    line_origin = [minPoint.x, minPoint.y]
                    line_trajectory = traj
                    dist = vector.lineDistanceToPoint([n.x, n.y], line_origin, line_trajectory)
                    #update cost
                    newCost = minPoint.cost + dist*n.intrinsicVal
                    if newCost < 0:
                        rospy.loginfo("Warning: found a negative cost")
                    # alternatively use the Sethian method to propagate the wavefront
                    #newCost = self.wavefrontUpdateValue(N, S, E, W) + n.intrinsicVal
                    if n.cost:
                        if newCost < n.cost:
                            n.cost = newCost
                            if n.cost < self.costThresh:
                                temp.append(n)  # put onto active list
                            else:
                                self.highCostList.append(n)
                            if n == self.startCell:
                                rospy.loginfo("Updated starting point cost to %f", n.cost)
                    else:
                        n.cost = newCost
                        if n.cost < self.costThresh:
                            temp.append(n)  # put onto active list
                        else:
                            self.highCostList.append(n)
                        if n == self.startCell:
                            rospy.loginfo("Updated starting point cost to %f", n.cost)
                #end for
            #end while
            #add the temp list to the active list, since the values for all the entries in temp have been updated
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
        while len(path) < maxPath and not util.closeToOne(currPos, goals):
            interpedGrad = self.interpolateGradientAtXY(currPos[0],currPos[1])
            #rospy.loginfo("path gradient: (%.2f, %.2f) of len %f", interpedGrad[0], interpedGrad[1], vector_length(interpedGrad))
            currPos = vector_add(currPos, vector_scale(interpedGrad, self.stepSize))
            self.path.append(currPos)
        #rospy.loginfo("Found goal. Path is:")
        v.vizConnectedPoints(path, color=[0,0,1])

