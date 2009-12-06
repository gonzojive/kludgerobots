from PIL import Image
import util
import math

class MapPoint:
    def __init__(self, xIndex, yIndex, xPos, yPos):
        self.xInd = xIndex
        self.yInd = yIndex
        self.x = xPos
        self.y = yPos
        self.intrinsicVal = 1
        self.cost = None

class DistanceMap:
    def __init__(self):
        fname = "data/gates-full-grayscale.png"
        self.im = Image.open(fname)
        print "Gates map loaded"
        [self.width, self.height] = self.im.size
        self.resolution = 0.1
        self.fWidth = self.width * self.resolution
        self.fHeight = self.height * self.resolution
        self.xMin = 0.0
        self.xMax = self.xMin + self.fWidth
        self.yMin = 0.0
        self.yMax = self.yMin + self.fHeight
        self.fSizeOfBucketDenomX = float(self.width)/float(self.xMax-self.xMin)
        self.fSizeOfBucketDenomY = float(self.height)/float(self.yMax-self.yMin)
        self.grid = list(self.im.getdata())
        self.gradientMap =  []
        self.maxCost = 1
        self.initializeGradientMap()
        self.go()
    def initializeGradientMap(self):
        rowNum =0
        curCol = []
        curX = 0.0
        for i in xrange(0, self.width):
            curY = 0.0
            for j in xrange(0, self.height):
                mp = MapPoint(i, j, curX, curY)
                curCol.append( mp )
                curY += 1
            curX += 1
            self.gradientMap.append(curCol)
            curCol = []
        self.activeList = []
        for col in self.gradientMap:
            for cell in col:
                if self.grid[cell.yInd*self.width + cell.xInd] < 127:
                    cell.cost = 0
                    self.activeList.append(cell)
    #function to find the 4 nearest neighbors
    def findNeighbors(self,point):
        neighbors = []
        x = point.xInd
        y = point.yInd
        if x > 0:
            neighbors.append(self.gradientMap[x-1][y])
        if x < self.width-1:
            neighbors.append(self.gradientMap[x+1][y])
        if y > 0:
            neighbors.append(self.gradientMap[x][y-1])
        if y < self.height-1:
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
        if i < 0 or i >= self.width or j < 0 or j >= self.height:
            return None
        cell = self.gradientMap[i][j]
        if cell.cost == None:
            return None
        return cell
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
            if len(roots) == 0:
                return math.sqrt(h * h * Fij * Fij) + Ta
            val = min(roots)
            assert(val >= 0)
            return val
        result = costFromCostsAtAdjCardinalDirections(minCost, adjCost)
        return result
    def go(self):
        iteration = 0
        # start from the goals on the active list, and propagate the values from there
        while self.activeList:
            iteration += 1
            if iteration % 10000 == 0:
                print "Iteration %d: active list has %d elements" % (iteration, len(self.activeList))
            #currentPoint is the point whose value we know, and
            #which will be used to propagate values
            currentPoint = self.activeList.pop(0)
            #calculate the neighbors of the point and update them
            neighbors = self.findNeighbors(currentPoint)
            
            for n in neighbors:
                # use the Sethian method to propagate the wavefront
                [N, S, E, W] = self.findCardinalNeighbors(n)
                newCost = self.wavefrontUpdateValue(N, S, E, W, n.intrinsicVal, n)
                if n.cost == None or newCost < n.cost:
                    n.cost = newCost
                    self.activeList.append(n)  # put onto active list
                    if newCost > self.maxCost:
                        self.maxCost = newCost
            #end for
        #end while
    def display(self):
        im = Image.new('L', (self.width, self.height))
        data = []
        for y in range(self.height):
            for x in range(self.width):
                cell = self.gradientMap[x][y]
                if not cell.cost:
                    val = 0
                else:
                    val = cell.cost*255/self.maxCost
                data.append(val)
        im.putdata(data)
        im.show()
    def save(self):
        im = Image.new('L', (self.width, self.height))
        im_raw = Image.new('L', (self.width, self.height))
        data = []
        raw_data = []
        for y in range(self.height):
            for x in range(self.width):
                cell = self.gradientMap[x][y]
                val = cell.cost*255/self.maxCost
                data.append(val)
                raw_data.append(cell.cost)
        im.putdata(data)
        im_raw.putdata(raw_data)
        im.save("gatesScaledDistanceMap.png")
        im_raw.save("gatesRawDistanceMap.png")

if __name__ == '__main__':
    dm = DistanceMap()
    dm.display()
    dm.save()
