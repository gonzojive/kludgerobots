from hw4 import readHw4Input
from circleDetect import findCircles
from visualizer import MapImage, MapFrame
import time
import wx
import hw4
import sys
import mapmodel
import particlefilter

class CircleDetectParametersFrame(wx.Frame):
    def __init__(self, parent, id, title, thresholdCallback=None, minThreshold=0.0, maxThreshold=.001):
        wx.Frame.__init__(self, parent, id, title, size = (200, 200))
        panel = wx.Panel(self, -1)
        box = wx.BoxSizer(wx.HORIZONTAL)
        self.cutoffSlider = wx.Slider(panel, 1, 0, 0, 1000, size=(120, -1))
        self.sliderValue = wx.StaticText(panel, -1, ".0001", (45, 25), style=wx.ALIGN_CENTRE)
        box.Add(self.cutoffSlider)
        box.Add(self.sliderValue)
        panel.SetSizer(box)
        panel.Bind(wx.EVT_SCROLL_CHANGED, self.onAdjust)

        self.thresholdCallback = thresholdCallback
        self.minThreshold = minThreshold
        self.maxThreshold = maxThreshold

        #self.Add(cutoffSlider)
        

    def onAdjust(self, event):
        f = float(self.cutoffSlider.GetValue()) / (self.cutoffSlider.GetMax() - self.cutoffSlider.GetMin())
        threshold = self.minThreshold + (self.maxThreshold - self.minThreshold) * f
        self.sliderValue.SetLabel("%f" % threshold)
        if self.thresholdCallback:
            self.thresholdCallback(self, threshold)

def main():
    [pose, laser] = readHw4Input()
    theMap = mapmodel.MapModel()
    pFilter = particlefilter.ParticleFilter(theMap, pose, laser)
    [newPose, mapLasers, objectLasers] = pFilter.run()
    print "Actual pose: ", pose.toStr()

    def findAndDrawCircles(cutoff=hw4.DEFAULT_CUTOFF):
        radius = .3
        mi = MapImage()
        circles = [c for c in findCircles(laser.points, radius, cutoff)]
        
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
            
        mi.image.save("output%i.png" % (1 if len(sys.argv) == 1 else int(sys.argv[1])), "PNG")
        return mi
            

    if hw4.interactive:
        app = wx.App()
        mframe = MapFrame(None, -1, "thingy", findAndDrawCircles())
        def callback(frame, cutoff):
            mframe.mapImage = findAndDrawCircles(cutoff)

        controlFrame = CircleDetectParametersFrame(mframe, -1, "Circle Detection Controls",  thresholdCallback=callback)
        controlFrame.Show()
        mframe.Show()
        app.MainLoop()
        
            


if __name__ == '__main__':
    main()
