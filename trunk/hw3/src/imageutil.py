import PIL
import PIL.Image
#import matplotlib
from pylab import randn, hist
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

    
# outputs a grayscale image, scaling the data according to its maximum
def showImageRowCol(data, width, height):
    im = PIL.Image.new('L', (width,height))  # 'P' for palettized
    maxData = max(data)
    dataScaled = [int(float(d) / float(maxData) * 255.0) for d in data]
    im.putdata(dataScaled)
    im.show()

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
    ax.set_ylim(0, 5.25)
    ax.grid(True)

    plt.show()
    
