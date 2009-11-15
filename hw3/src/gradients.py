

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
    def __init__(self,xMapSize,yMapSize,obstacleMap,laserReadings):
        self.gradientMap =  MapPoint[xMapSize][yMapSize]
        self.ActiveList = []
    def getGradientAtXY(self,x,y):
        return self.gradientMap[x][y].gradient
        
    def updateValuesAtXY(self,x,y,targetX,targetY):
        #find distance from target
        dist = abs(x-targetX) +abs(y-targetY)
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
        rowNum =0
        for columns =0:yMapSize
            self.gradientMap[rowNum][column]
        self.gradientMap[x][y].intrinsicVal = self.gradientMap[targetX][targetY].intrinsicVal*1/dist;
        #change goal value as a function of Euclidean distance 
        
    #function to find and set the gradient value at a point (x,y) using the values at its neighbors
    def findGradientAtXY(self,x,y):
        #check if the neighbor values are updated    
        for xdash in -1:1:
            for ydash in -1:1:
                if self.gradientMap[xdash][ydash].gradient !=-1:
                    #this value is updated. Update the value at (x,y) using this point now
                    updateValuesAtXY(x,y,xdash,ydash)
