

#MapPoint is a structure to hold the gradient field values for all points in the map. gradient is the direction of the gradient at each (x,y) point. intrinsicVal is the distance to the nearest obstacle and goalVal is the distance to the goal
class MapPoint:
    #initialize all values to -1 to show that they are not initialized/visited yet
    def __init__(self):
        self.gradient =0
        self.intrinsicVal = 0
        self.goalVal = 0
    def set(self,gradient,intrinsicVal,goalVal):
        self.gradient = gradient
        self.intrinsicVal = intrinsicVal
        self.goalVal = goalVal
        
        
#GradientField is a class which creates and updates the gradient field for the map. It is generalized to perform global and local gradient updates
class GradientField:
    def __init__(self,xMapSize,yMapSize,goals,laserReadings):
        self.gradientMap =  MapPoint[xMapSize][yMapSize]
        self.ActiveList = []
        for goal in goals:
            self.ActiveList.append(goal)
    def getGradientAtXY(self,x,y):
        return self.gradientMap[x][y].gradient
        
    def setObstacles(self):
        #find obstacle costs by loading the marshalled map into the matrix
        fname = sys.path[0] + '/../' + 'myGatesMapDist.marshal'
        try:
            stream = file(fname, 'r')
            result = marshal.load(stream)
            #result now has the map, but in a 1-D array,not as a matrix
            rospy.loginfo("Map was successfully loaded from " + fname);
        except IOError, exc:
            rospy.loginfo("Map failed to load from " + fname);
            result = self.computeDistanceFromObstacleGrid()
            stream = file(fname, 'w')
            marshal.dump(result, stream)
            sys.exit(-1)
            
        #now copy into the 2D Matrix
        rowNum =0
        for i =0:len(result):
            if i % yMapSize == 0:
                rowNum = rowNum +1
                colNum =0
            self.gradientMap[rowNum][colNum].intrinsicVal = result[rowNum +colNum]
            colNum = colNum +1
                    
    #function to find and set the gradient value at a point (x,y) using the values at its neighbors
    def propagateGoalValues(self):
         #start from the goals on the active list, and propagate the values from there
         
       
