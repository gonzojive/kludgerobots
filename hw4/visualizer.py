import PIL
import PIL.Image
import PIL.ImageDraw
from PIL import Image,ImageDraw
import util
from vector import *
import wx
from math import *

class MapFrame(wx.Frame):
    def __init__(self, parent, id, title, mapImage):
        wx.Frame.__init__(self, parent, id, title, size = vector_add(mapImage.image.size, [0, 0]))
        self.bitmap = mapImage.bitmap
        self.Centre()
        wx.EVT_PAINT(self, self.OnPaint)
        #self.Add(cutoffSlider)

    def set_mapImage(self, mi):
        self.bitmap = mi.bitmap
        self.Refresh()
        

    mapImage = property(None, set_mapImage)
          
    def OnPaint(self, event):
        dc = wx.PaintDC(self)
        dc.DrawBitmap(self.bitmap, 10, 30)

class MapImage:
    def __init__(self,filename="data/gates-full-grayscale.png"):
        self.filename = filename
        fileImage = PIL.Image.open(self.filename)
        scaleFactor = 2.0
        self.pixelsPerMapUnit = 10.0 * scaleFactor
        self.image = fileImage.resize(map(int, vector_scale(fileImage.size, scaleFactor))).convert("RGB")
        self.draw = ImageDraw.Draw(self.image)


    def get_bitmap(self):
        """
        wx-compatible bitmap image
        """
        def pilToImage(pil,alpha=True):
            if alpha:
                image = wx.EmptyImage( *pil.size )
                image.SetData( pil.convert( "RGB").tostring() )
                image.SetAlphaData(pil.convert("RGBA").tostring()[3::4])
            else:
                image = wx.EmptyImage(pil.size[0], pil.size[1])
                new_image = pil.convert('RGB')
                data = new_image.tostring()
                image.SetData(data)
                return image

        return pilToImage(self.image, False).ConvertToBitmap()

    
    
    def get_imageBounds(self):
        return self.image.size

    imageBounds = property(get_imageBounds, None)
    bitmap = property(get_bitmap, None)

    def pointToImageXY(self, point):
        return map(int, vector_scale(point, self.pixelsPerMapUnit))
            
    #function to draw a circle given a center,a radius and an image to overlay on
    def drawCircle(self, center, radius, fill=255):
        #the circle is drawn as an ellipse within a bounding box with(left,upper,right,lower) coordinates given.
        #convert the (center,radius) to this system now
        #ASSUMPTION!! -  the center and the radius are within the bounds of the image!!
        center = self.pointToImageXY(center)
        radius = int(float(radius) * self.pixelsPerMapUnit)
        left  = center[0] - radius;
        upper = center[1] - radius;
        right = center[0] + radius;
        lower = center[1] + radius;
        self.draw.ellipse((left,upper,right,lower), fill=fill) # Draw a circle

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
        else:
            return False
            
    #function to show an arrow at the given point given an angle
    def showArrow(self,rawx,rawy,theta):
        #imagePath = "/home/ckalyan/Desktop/cs225b_code/hw4/data/gates-full-stage.png"):
        #arrow starts at point(x,y) and extends a few pixels ahead at the given angle
        
        [x,y] = self.pointToImageXY([rawx,rawy])
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
        self.image.show()
