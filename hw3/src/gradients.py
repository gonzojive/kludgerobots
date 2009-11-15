import rospy
import mapmodel
import imageutil

ROBOT_RADIUS = 0.15
DISTANCE_CHANGE_POINT = 1.0
MAX_OBSTACLE_DISTANCE = 1.2

#MapPoint is a structure to hold the gradient field values for all points in the map. gradient is the direction of the gradient at each (x,y) point. intrinsicVal is the distance to the nearest obstacle and goalVal is the distance to the goal
class MapPoint:
    #initialize all values to -1 to show that they are not initialized/visited yet
    def __init__(self, xIndex, yIndex):
        self.x = xIndex
        self.y = yIndex
        self.gradient = []
        self.intrinsicVal = 0
        self.cost = None
        
        
#GradientField is a class which creates and updates the gradient field for the map. It is generalized to perform global and local gradient updates
class GradientField:
    def __init__(self, cellSpacing, distanceMap, laserReadings = None):
        # constants for calculating the intrinsic (obstacle distance) cost
        self.robotRadius = ROBOT_RADIUS
        self.maximumDistance = MAX_OBSTACLE_DISTANCE
        self.changeDistance = DISTANCE_CHANGE_POINT
        self.intrinsicMaxValue = 10000.0
        self.intrinsicStartValue = 100.0
        self.intrinsicChangeValue = 50.0
        self.intrinsicEndValue = 1.0
        self.shallowSlope = (self.intrinsicChangeValue - self.intrinsicStartValue) / (self.changeDistance - self.robotRadius)
        self.steepSlope = (self.intrinsicEndValue - self.intrinsicChangeValue) / (self.maximumDistance - self.changeDistance)
        # data members
        self.mapWidth = distanceMap.meta.width
        self.mapHeight = distanceMap.meta.height
        self.spacing = cellSpacing
        self.gridWidth = int(self.mapWidth / cellSpacing)
        self.gridHeight = int(self.mapHeight / cellSpacing)
        self.gradientMap =  []
        self.initializeGradientMap(distanceMap)

    def setGoals(self, goals):
        for row in self.gradientMap:
            for point in row:
                point.cost = None
        self.activeList = []
        for g in goals:
            cell = self.cellNearestXY(g.x, g.y)
            cell.cost = 0
            self.activeList.append(cell)
 
    def cellNearestXY(self, x, y):
        xIndex = int( float(x)/self.spacing + 0.5 )
        yIndex = int( float(y)/self.spacing + 0.5 )
        return self.gradientMap[xIndex][yIndex]

    def interpolateGradientAtXY(self, x, y):
        gridX = float(x)/self.spacing
        gridY = float(y)/self.spacing
        sw = self.gradientMap[int(gridX)][int(gridY)].gradient
        se = self.gradientMap[int(gridX+1)][int(gridY)].gradient
        nw = self.gradientMap[int(gridX)][int(gridY+1)].gradient
        ne = self.gradientMap[int(gridX+1)][int(gridY+1)].gradient
        alphaX = float(gridX - int(gridX))/self.spacing
        alphaY = float(gridY - int(gridY))/self.spacing
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
        for i in xrange(0, self.gridWidth):
            curY = 0.0
            for j in xrange(0, self.gridHeight):
                mp = MapPoint(i, j)
                if distanceMap.pointInBounds([curX, curY]):
                    mp.intrinsicVal = self.intrinsicFunc(distanceMap.distanceFromObstacleAtPoint([curX, curY]))
                else:
                    mp.intrinsicVal = self.intrinsicMaxValue
                curCol.append( mp )
                curY += self.spacing
            curX += self.spacing
            self.gradientMap.append(curCol)
            curCol = []
            rospy.loginfo("%0.2f percent done", float(i)/float(self.gridWidth)*100.0)
        vals = []
        for c in self.gradientMap:
            for d in c:
                data = 0
                if d.intrinsicCost > 100:
                    data = 100
                else:
                    data = d.intrinsicCost
                vals.append(data)
        

    # intrinsicFunc()
    # convert distances into intrinsic values, using the Konolidge paper as a model
    # very large value when distance < robot radius
    # gradually sloping down from there until a certain point
    # steep slope from there until the maximum distance
    # stays at end value (1.0) past there
    def intrinsicFunc(self, distance):
        if distance < self.robotRadius:
            return self.intrinsicMaxValue
        elif distance < self.changeDistance:
            return self.intrinsicStartValue + (self.shallowSlope * (distance - self.robotRadius))
        elif distance < self.maximumDistance:
            return self.intrinsicChangeValue + (self.steepSlope * (distance - self.changeDistance))
        else:
            return self.intrinsicEndValue
                    
    # calculateCosts()
    # assumes the active list has been set, and goal costs have been set to 0
    def calculateCosts(self):
         # start from the goals on the active list, and propagate the values from there
         pass


