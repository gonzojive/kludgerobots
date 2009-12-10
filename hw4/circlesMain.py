from hw4 import readHw4Input
from circleDetect import findCircles
from visualizer import MapImage, MapFrame
import time
import wx
import hw4
import sys
import mapmodel
import particlefilter
import time
import math
import os
from vector import *
import pose as posemodule
import util


curTime = 0
timeDiff = 0

TRUE_POSITIVE_DISTANCE_CUTOFF = .16 # must estimate a circle within .1 meters for it to count as a "true positive"

def updateTimes():
    global curTime
    global timeDiff
    timeDiff = time.clock() - curTime
    curTime += timeDiff

class CircleDetectParametersFrame(wx.Frame):
    def __init__(self, parent, id, title, thresholdCallback=None, minThreshold=0.0, maxThreshold=.002):
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

    def set_threshold(self, threshold):
        f = (threshold - self.minThreshold) / (self.minThreshold + self.maxThreshold)
        v = int(f * (self.cutoffSlider.GetMax() - self.cutoffSlider.GetMin()) + self.cutoffSlider.GetMin())
        self.sliderValue.SetLabel("%f" % f)
        self.cutoffSlider.SetValue(v)

    threshold = property(None, set_threshold)

    def onAdjust(self, event):
        f = float(self.cutoffSlider.GetValue()) / (self.cutoffSlider.GetMax() - self.cutoffSlider.GetMin())
        threshold = self.minThreshold + (self.maxThreshold - self.minThreshold) * f
        self.sliderValue.SetLabel("%f" % threshold)
        if self.thresholdCallback:
            self.thresholdCallback(self, threshold)

def coalesceCircles(circles):
    #print "Starting with %d circles" % len(circles)
    #print [[c.center, c.error] for c in circles]
    circleCopy = circles[:]
    for c in circleCopy:
        for c2 in circleCopy:
            if c == c2:
                continue
            dx = c2.center[0] - c.center[0]
            dy = c2.center[1] - c.center[1]
            if math.sqrt( dx*dx + dy*dy ) < 0.75*(c2.radius + c.radius):
                badCircle = c if c.error > c2.error else c2
                try:
                    circles.remove(badCircle)
                except:
                    pass
    #print "Finished with %d circles" % len(circles)
    #print [[c.center, c.error] for c in circles]
    return circles

def outputArffHeader(f):
    f.write("""% CS225b final project training input file for WEKA
@relation pucks
@attribute 'mean-squared-error' real
@attribute 'mean-1norm-error' real
@attribute 'max-error' real
@attribute '2max-error' real
@attribute '3max-error' real
@attribute 'cluster1-dist' real
@attribute 'cluster2-dist' real
%@attribute 'distance-to-center' real
@attribute 'num-circumference-points' real
@attribute 'circumference-degrees' real
@attribute 'class' {'puck','not-puck'}
@data
""")
    

def main():
    updateTimes()
    print "Reading input ...",
    sys.stdout.flush()
    [pose, laser, actualCircles] = readHw4Input()
    theMap = mapmodel.MapModel()
    updateTimes()
    print "done (%0.2f s)" % (timeDiff)
    print "Running filter ...",
    sys.stdout.flush()
    pFilter = particlefilter.ParticleFilter(theMap, pose, laser)
    [pose, mapLasers, objectLasers] = pFilter.run()
    updateTimes()
    print "done (%0.2f s)" % (timeDiff)
    print "Actual pose: ", pose.toStr()
    outPose = mapmodel.mapToWorld([pose.x, pose.y, pose.theta])
    print "In input coordinate system: ", posemodule.Pose(outPose[0], outPose[1], util.d2r(outPose[2])).toStr()

    outputtedTrainingData = [False] # this is an array because Python closures suxxx

    def findAndDrawCircles(cutoff=hw4.DEFAULT_CUTOFF, training=False):
        updateTimes()
        print "Finding circles ...",
        sys.stdout.flush()
        radius = .25
        mi = MapImage()
        circleDetectPoints = objectLasers # laser.points
        circles = [c for c in findCircles(circleDetectPoints, radius, cutoff,training=training)]
	circles = coalesceCircles(circles)
        
        if len(circles) > 0:
            minError = min(map(lambda x : x.error, circles))
            maxError = max(map(lambda x : x.error, circles))
            #print "Found %i circle centers with radius %f" % (len(circles), radius)
            for circle in circles:
                center = pose.inMapFrame(circle.center)
                errPercentile = 1.0 - (circle.error - minError) / (maxError - minError + .000001)
                color = (0,int(255.0 * errPercentile),0)
                mi.drawCircle(center, circle.radius, fill=color)
            
        for pt in mapLasers:
            mi.drawCircle(pose.inMapFrame(pt), .05, fill=(0,0,255))
        for pt in objectLasers:
            mi.drawCircle(pose.inMapFrame(pt), .05, fill=(255, 0, 0))
        mi.drawCircle(pose.inMapFrame([0,0]), .3, fill=(255, 0, 255))
        if actualCircles:
            # draw them
            for c in actualCircles:
                mi.drawCircle(pose.inMapFrame([c[0], c[1]]), 0.3, fill=(255, 127, 0))

            if not outputtedTrainingData[0]:
                outputtedTrainingData[0] = True
                # now output training data to train.arff
                if os.path.exists("train.arff"):
                    f = open("train.arff", 'a')
                else:
                    f = open("train.arff", 'w')
                    outputArffHeader(f)

                f.write("% circles for single session:\n")

                for c in circles:
                    tpDistance = min(map(lambda tp: vector_distance(c.center, tp), actualCircles))
                    c.knownClass = "puck" if tpDistance <= TRUE_POSITIVE_DISTANCE_CUTOFF else "not-puck"
                    f.write("%s%s\n" % (c.arffLine(), "")) #"%f away from nearest actual puck" % tpDistance))

                f.close()

        #print "drawing circle at:",pose.inMapFrame([0,0])
        #draw the arrow to show the pose
        temp = pose.inMapFrame([0,0])
        mi.showArrow(temp[0],temp[1],-pose.theta)    
        try:
            if len(sys.argv) == 1:
                i = 1
            else:
                i = int(sys.argv[1])
            fname = "output%i.png" % (i)
        except:
            fname = "output.png"
       
        mi.image.save(fname, "PNG")
        updateTimes()
        print "done (%0.2f s)" % (timeDiff)
        #mi.show()
        return mi

    # if we are training, then we find all circles regardless of cutoff and output them to
    # the training data file
    if actualCircles:
        findAndDrawCircles(cutoff=None,training=True)#hw4.DEFAULT_CUTOFF * 3.0)

    if hw4.interactive:
        app = wx.App()
        mframe = MapFrame(None, -1, "thingy", findAndDrawCircles())
        def thresholdCallback(frame, cutoff):
            mframe.mapImage = findAndDrawCircles(cutoff)

        controlFrame = CircleDetectParametersFrame(mframe, -1, "Circle Detection Controls",  thresholdCallback=thresholdCallback)
        controlFrame.threshold = hw4.DEFAULT_CUTOFF
        controlFrame.Show()
        mframe.Show()
        app.MainLoop()
    else:
        findAndDrawCircles()

    
        
            


if __name__ == '__main__':
    main()
