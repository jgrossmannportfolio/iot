#!/usr/bin/env python

#import MathUtil
import re
import numpy as np
import matplotlib.pyplot as plt
import MathUtil

delta_t = .10 # seconds


# read in text file
with open ("accelData.dat", "r") as myfile:
    data = myfile.read().replace('\n', '')


# delimiters
new = re.split(', |\[|\, |\,|\]', data)



def create3plots(yticks,plotData):
    # plotData must be in the form of [xdataArray, ydataArray, zdataArray]

    # first sub plotting
    # - configure the axes sizes
    ymin0 = min(plotData[0])
    ymin1 = min(plotData[1])
    ymin2 = min(plotData[2])

    ymax0 = max(plotData[0])
    ymax1 = max(plotData[1])
    ymax2 = max(plotData[2])

    ymax = max([ymax0,ymax1,ymax2])
    ymin = min([ymin0,ymin1,ymin2])
    xmax = timestamp[-1]
    xmin = 0
    axesRange = [xmin, xmax, ymin, ymax]
    
    yTicks = (ymax - ymin)/yticks 
    fig = plt.figure()
    
    ax = fig.add_subplot(131)
    fig.subplots_adjust(top=0.85)
    ax.set_title(r'$X$')

    ax.set_xlabel(r'Time ($s$)')

    ax.set_xticks(np.arange(xmin,xmax,delta_t))
    ax.set_yticks(np.arange(ymin,ymax,yTicks))

    ax.plot(timestamp, plotData[0], 'o')

    ax.axis(axesRange)
    ax.grid(True)
    #ax2.set_xticklabels([])

    # second subplot
    # - configure the axes sizes

    ax2 = fig.add_subplot(132)
    fig.subplots_adjust(top=0.85)
    ax2.set_title(r'$Y$')
    
    ax2.set_xlabel(r'Time ($s$)')

    ax2.set_xticks(np.arange(xmin,xmax,delta_t))
    ax2.set_yticks(np.arange(ymin,ymax,yTicks))

    ax2.plot(timestamp, plotData[1], 'o', color='red')

    ax2.axis(axesRange)
    #plt.grid()
    ax2.grid(True)
    ax2.set_yticklabels([])

    # third subplot
    # - configure the axes sizes

    ax3 = fig.add_subplot(133)
    fig.subplots_adjust(top=0.85)
    ax3.set_title(r'$Z$')

    ax3.set_xlabel(r'Time ($s$)')

    ax3.set_xticks(np.arange(xmin,xmax,delta_t))
    ax3.set_yticks(np.arange(ymin,ymax,yTicks))

    ax3.plot(timestamp, plotData[2], 'o', color='green')

    ax3.axis(axesRange)
    ax3.grid(True)
    ax3.set_yticklabels([])

    return ax, fig

def getVeloAndDispLists(aList):
  vList = [0.0]
  dList = [0.0]
  for i in (range(len(aList)-1)):
    dPoint,vPoint = MathUtil.displacement(dList[-1], vList[-1], aList[i], delta_t)
    dList.append(dList[-1]+dPoint)
    vList.append(vPoint)
  return dList, vList




# Append incoming data into ax, ay, az, aMag lists
count = 0
axData = []
ayData = []
azData = []
aDataMag = []
for x in (range(len(new))):
    if new[x] == '':
        count -= 1
    elif count == 0:
        axData.append(float(new[x]))
    elif count == 1:
        ayData.append(float(new[x]))
    elif count == 2:
        azData.append(float(new[x]))
    elif count == 3:
        aDataMag.append(float(new[x]))
        count = -1
    count += 1

# timestamp list for the x axis
timestamp = [0.0]
for i in (range(len(axData)-1)):
    timestamp.append(timestamp[i] + delta_t) 

# Calculate velocity and displacement lists
dxData = []
dyData = []
dzData = []
vxData = []
vyData = []
vzData = []

dxData, vxData = getVeloAndDispLists(axData)
dyData, vyData = getVeloAndDispLists(ayData)
dzData, vzData = getVeloAndDispLists(azData)

# Create Accel xyz plots
#   -set axes
yTicks = 10 # number of ticks
accelPlotData = [axData, ayData, azData]

ayHandle, figAccel = create3plots(yTicks,accelPlotData)
figAccel.suptitle('Acceleration Data', fontsize=14, fontweight='bold')
ayHandle.set_ylabel(r'Acceleration ($m/s^2$)')


# Create Velo xyz plots
veloPlotData = [vxData, vyData, vzData]

vyHandle, figVelo = create3plots(yTicks,veloPlotData)
figVelo.suptitle('Velocity Calculation', fontsize=14, fontweight='bold')
vyHandle.set_ylabel(r'Velocity ($m/s$)')


# Create Displacement xyz plots
dispPlotData = [dxData, dyData, dzData]

dyHandle, figDisp = create3plots(yTicks,dispPlotData)
figDisp.suptitle('Displacement Calculation', fontsize=14, fontweight='bold')
dyHandle.set_ylabel(r'Displacement ($m$)')



plt.show()






