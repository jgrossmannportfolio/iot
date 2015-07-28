#!/usr/bin/env python

#import MathUtil
import re
import numpy as np
import matplotlib.pyplot as plot
import MathUtil

import math

delta_t = .0125 # seconds


# read in data file
with open ("accelData.dat", "r") as myfile:
    data = myfile.read().replace('\n', '')

# delimiters
new = re.split(', |\[|\, |\,|\]', data)

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

dxData, vxData = MathUtil.getVeloAndDispLists(axData, timestamp)
dyData, vyData = MathUtil.getVeloAndDispLists(ayData, timestamp)
dzData, vzData = MathUtil.getVeloAndDispLists(azData, timestamp)

# Create Accel xyz plots
#   -set axes
accelPlotData = [axData, ayData, azData]
plotTitle = 'Acceleration Data'
ylabel = r'Acceleration ($m/s^2$)'
xlabel = r'Time ($s$)'

figAccel = MathUtil.create3plots(timestamp, accelPlotData, plotTitle, xlabel, ylabel)

# Create Velo xyz plots
veloPlotData = [vxData, vyData, vzData]
plotTitle = 'Velocity Calculation'
ylabel = r'Velocity ($m/s$)'
xlabel = r'Time ($s$)'

figVelo = MathUtil.create3plots(timestamp, veloPlotData, plotTitle, xlabel, ylabel)

# Create Displacement xyz plots
dispPlotData = [dxData, dyData, dzData]
plotTitle = 'Displacement Calculation'
ylabel = r'Displacement ($m$)'
xlabel = r'Time ($s$)'

figDisp = MathUtil.create3plots(timestamp, dispPlotData, plotTitle, xlabel, ylabel)



plot.show() # renders all three plots






