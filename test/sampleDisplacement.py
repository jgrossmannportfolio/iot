#!/usr/bin/env python

#import MathUtil
import re
import numpy as np
import matplotlib.pyplot as plot
import MathUtil

import math
import filters

delta_t = .0125 # seconds

# --> Read in simulated putt:
#       sampleDisplacement.dat
#       sampleGyroDisplacement.dat

# read in data file
with open ("sampleDisplacement.dat", "r") as myfile:
    data = myfile.read().replace('\n', '')

# delimiters
new = re.split(', |\[|\, |\,|\]', data)

# Append incoming data into ax, ay, az, aMag lists
count = 0
dxData = []
dyData = []
dzData = []
for x in (range(len(new))):
    if new[x] == '':
        count -= 1
    elif count == 0:
        dxData.append(float(new[x]))
    elif count == 1:
        dyData.append(float(new[x]))
    elif count == 2:
        count = -1
        dzData.append(float(new[x]))
    count += 1



# timestamp list for the x axis
timestamp = [0.0]
for i in (range(len(dxData)-1)):
    timestamp.append(timestamp[i] + delta_t) 


# Create Displacement xyz plots
dispPlotData = [dxData, dyData, dzData]
plotTitle = 'Simulated Putt Displacement'
ylabel = r'Displacement ($m$)'
xlabel = r'Time ($s$)'

figDisp = MathUtil.create3plots(timestamp, dispPlotData, plotTitle, xlabel, ylabel)



plot.show() # renders all three plots






