import PIL
import PIL.Image
import PIL.ImageDraw
from PIL import Image,ImageDraw
#import matplotlib
from pylab import randn, hist
from pylab import *

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import util
import numpy
#import scipy.io
    
# outputs a grayscale image, scaling the data according to its maximum
def showImageRowCol(data, width, height):
    im = PIL.Image.new('L', (width,height))  # 'P' for palettized
    maxData = max(data)
    if maxData > 0:
        dataScaled = [int(float(d) / float(maxData) * 255.0) for d in data]
    else:
        dataScaled = [0] * len(data)
    im.putdata(dataScaled)
    im.show()

    
# outputs a color image, scaling the data according to its maximum
def showColorImageRowCol(data, width, height):
    im = PIL.Image.new('RGB', (width,height))  # 'P' for palettized
    maxData = max(data)
    if maxData > 0:
        dataScaled = [int(float(d) / float(maxData) * 255.0) for d in data]
    else:
        dataScaled = [0] * len(data)
    im.putdata(dataScaled)
    im.show()
    
    
#function to draw a circle given a center,a radius and an image to overlay on
def showCircleAtPoint(centerx,centery,radius,imagePath = "data/gates-full-grayscale.png"):#"/home/ckalyan/Desktop"):
    im = PIL.Image.open(imagePath)
    draw = ImageDraw.Draw(im) # Create a draw object
    #the circle is drawn as an ellipse within a bounding box with(left,upper,right,lower) coordinates given. 
    #convert the (center,radius) to this system now
    #ASSUMPTION!! -  the center and the radius are within the bounds of the image!!
    left  = centerx - radius;
    upper = centery - radius;
    right = centerx + radius;
    lower = centery + radius;
    draw.ellipse((left,upper,right,lower), fill=(255,0,0)) # Draw a circle
    im.show()
    print "drawing circle"


def showArrowAtPointAndAngle(x,y,angle,imagePath = "data/gates-full-grayscale.png"):
    im = PIL.Image.open(imagePath)
    draw = ImageDraw.Draw(im)
    #arrow starts at point(x,y) and extends 2 pixels ahead at the given angle
    print "drawing line"
    theta = util.d2r(angle) 
    length = 20
    endx = x+ length*cos(theta)
    endy = y+ length*sin(theta)
    draw.line((x,y,endx,endy),fill =128)
    
    Par = length/4
    cosy = cos(theta)
    siny = sin(theta)

    draw.line((endx, endy, endx + int(-Par * cosy - (Par / 2.0 * siny)), endy + int(-Par * siny + (Par / 2.0 * cosy))),fill =128)
    draw.line((endx,endy,endx + int(-Par * cosy + (Par / 2.0 * siny)),endy - int(Par / 2.0 * cosy + Par * siny )),fill =128)
    im.show()

    
def showImageRowCol2(data, width, height):
    im = PIL.Image.new('L', (width,height))  # 'P' for palettized
    maxData = max(data)
    minData = min(data)
    if maxData > 0:
        dataScaled = [int(float(d) / float(maxData - minData) * 255.0) for d in data]
    else:
        dataScaled = [0] * len(data)
    im.putdata(dataScaled)
    im.show()
#    outputMatrix(data, width, height)

#def outputMatrix(data, width, height, filename='/tmp/arrdata.mat', i=""):
    # output the matlab matrix
#    arr = numpy.array(data)
#    arr = arr.reshape((height, width))
#    scipy.io.savemat(filename, mdict={"arr%i" % i: arr})

def showMapImageRowCol(data, width, height):
    im = PIL.Image.new('L', (width,height))  # 'P' for palettized
    def clampValue(val):
        if val > 2:
            val = 2.0
        return val

    data = [clampValue(d) for d in data]

    maxData = max(data)
    def normalizeValue(val):
        int(255.0 * float(val)  / float(maxData))
        
    dataScaled = [normalizeValue(d) for d in data]
    im.putdata(dataScaled)
    im.show()
    
    im.save("/tmp/thingy.png", "PNG")
            


def showHistPlotOfNums(nums, numFields=50):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # the histogram of the data
    n, bins, patches = ax.hist(nums, numFields, facecolor='green', alpha=0.85, normed=1)

    # hist uses np.histogram under the hood to create 'n' and 'bins'.
    # np.histogram returns the bin edges, so there will be 50 probability
    # density values in n, 51 bin edges in bins and 50 patches.  To get
    # everything lined up, we'll compute the bin centers
    #bincenters = 0.5*(bins[1:]+bins[:-1])
    # add a 'best fit' line for the normal PDF
    #y = mlab.normpdf( bincenters, mu, sigma)
    #l = ax.plot(bincenters, y, 'r--', linewidth=1)

    ax.set_xlabel('Numbers')
    ax.set_ylabel('Probability')
    #ax.set_title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
    ax.set_xlim(min(nums), max(nums))
    ax.set_ylim(0, 1.0)
    ax.grid(True)

    plt.show()
    
