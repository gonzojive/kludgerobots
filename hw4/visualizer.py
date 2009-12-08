import PIL
import PIL.Image
import PIL.ImageDraw
from PIL import Image,ImageDraw
import util


class visualizer:
    def __init__(self,filename):
        self.filename = filename
        self.im = PIL.Image.open(self.filename)
        self.imageBounds = im.size
        self.draw = ImageDraw.Draw(self.im)
            
    #function to draw a circle given a center,a radius and an image to overlay on
    def showCircleAtPoint(self,centerx,centery,radius):
        #the circle is drawn as an ellipse within a bounding box with(left,upper,right,lower) coordinates given. 
        #convert the (center,radius) to this system now
        
        left  = centerx - radius;
        upper = centery - radius;
        right = centerx + radius;
        lower = centery + radius;
        flag1 = self.checkWithinBounds(left,upper)
        flag2 = self.checkWithinBounds(right,lower)
        if flag1 == True and flag2 ==True:
            self.draw.ellipse((left,upper,right,lower), fill=(255,0,0)) # Draw a circle

    def checkWithinBounds(self,x,y):
        if x < self.imageBounds[0] and y < self.imageBounds[1]:
            return True
        else
            return False
            
    #function to show an arrow at the given point given an angle
    def showArrow(self,x,y,angle):
        #imagePath = "/home/ckalyan/Desktop/cs225b_code/hw4/data/gates-full-stage.png"):
        #arrow starts at point(x,y) and extends a few pixels ahead at the given angle
        theta = util.d2r(angle) 
        length = 20
        endx = x+ length*cos(theta)
        endy = y+ length*sin(theta)
        
        if self.checkWithinBounds(x,y) == True and self.checkWithinBounds(endx,endy) ==True:
            self.draw.line((x,y,endx,endy),fill =128)
        
        Par = length/4
        cosy = cos(theta)
        siny = sin(theta)
        
        arrowx =  endx + int(-Par * cosy - (Par / 2.0 * siny))
        arrowy =  endy + int(-Par * siny + (Par / 2.0 * cosy))
        if self.checkWithinBounds(endx,endy) == True and self.checkWithinBounds(arrowx,arrowy) ==True:
            self.draw.line((endx,endy,arrowx,arrowy),fill =128)
        
        arrowx =  endx + int(-Par * cosy + (Par / 2.0 * siny))
        arrowy =  endy - int(Par / 2.0 * cosy + Par * siny )
        if self.checkWithinBounds(endx,endy) == True and self.checkWithinBounds(arrowx,arrowy) == True:
            self.draw.line((endx,endy,arrowx,arrowy),fill =128)
        
    def show(self):
        self.im.show()
