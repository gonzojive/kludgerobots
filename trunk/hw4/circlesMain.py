from hw4 import readHw4Input
from circleDetect import findCircles
from visualizer import MapImage, MapFrame
import time
import wx
import hw4
import sys

def main():
    [pose, laser] = readHw4Input()
    for radius in [.3]:
        mi = MapImage()
        circles = [c for c in findCircles(laser.points, radius)]
        
        if len(circles) > 0:
            minError = min(map(lambda x : x.error, circles))
            maxError = max(map(lambda x : x.error, circles))
            print "Found %i circle centers with radius %f" % (len(circles), radius)
            for circle in circles:
                center = pose.inMapFrame(circle.center)
                errPercentile = 1.0 - (circle.error - minError) / (maxError - minError + .000001)
                color = (0,int(255.0 * errPercentile),0)
                mi.drawCircle(center, circle.radius, fill=color)
            
        for pt in laser.points:
            mi.drawCircle(pose.inMapFrame(pt), .05, fill=(255,0,0))
            
        #mi.show()
        mi.image.save("output%i.png" % (1 if len(sys.argv) == 1 else int(sys.argv[1])), "PNG")
        if hw4.interactive:
            app = wx.App()
            frame = MapFrame(None, -1, "thingy", mi)
            frame.Show()
            app.MainLoop()
        
            


if __name__ == '__main__':
    main()
