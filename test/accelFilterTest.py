#!/usr/bin/env python

#import MathUtil
import re
import numpy as np
import matplotlib.pyplot as plot
import MathUtil

import math
import filters

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


f_axData = filters.lowpass(axData, 0.09)
f_ayData = filters.lowpass(ayData, 0.09)
f_azData = filters.lowpass(azData, 0.09)

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

#dxData, vxData = MathUtil.getVeloAndDispLists(axData, timestamp)
dxData, vxData = MathUtil.interpolateDisplacement(axData, delta_t)
dyData, vyData = MathUtil.getVeloAndDispLists(ayData, timestamp)
dzData, vzData = MathUtil.getVeloAndDispLists(azData, timestamp)

f_dxData, f_vxData = MathUtil.interpolateDisplacement(f_axData, delta_t)
f_dyData, f_vyData = MathUtil.getVeloAndDispLists(f_ayData, timestamp)
f_dzData, f_vzData = MathUtil.getVeloAndDispLists(f_azData, timestamp)

# Create Accel xyz plots
#   -set axes
accelPlotData = [axData, ayData, azData]
f_accelPlotData = [f_axData, f_ayData, f_azData]
plotTitle = 'Acceleration Data'
ylabel = r'Acceleration ($m/s^2$)'
xlabel = r'Time ($s$)'

figAccel = MathUtil.create3plots(timestamp, accelPlotData, f_accelPlotData, plotTitle, xlabel, ylabel)

# Create Velo xyz plots
veloPlotData = [vxData, vyData, vzData]
f_veloPlotData = [f_vxData, f_vyData, f_vzData]
plotTitle = 'Velocity Calculation'
ylabel = r'Velocity ($m/s$)'
xlabel = r'Time ($s$)'

figVelo = MathUtil.create3plots(timestamp, veloPlotData, f_veloPlotData, plotTitle, xlabel, ylabel)

# Create Displacement xyz plots
dispPlotData = [dxData, dyData, dzData]
f_dispPlotData = [f_dxData, f_dyData, f_dzData]
plotTitle = 'Displacement Calculation'
ylabel = r'Displacement ($m$)'
xlabel = r'Time ($s$)'

figDisp = MathUtil.create3plots(timestamp, dispPlotData, f_dispPlotData, plotTitle, xlabel, ylabel)



plot.show() # renders all three plots






