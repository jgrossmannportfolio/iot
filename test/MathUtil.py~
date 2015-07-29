# Math utility functions
import math
import matplotlib.pyplot as plot
import numpy as np

PI = 3.1415926536

# Global functions/hex and byte conversions
tosigned = lambda n: float(n-0x10000) if n>0x7fff else float(n)
tosignedbyte = lambda n: float(n-0x100) if n>0x7f else float(n)

def RtoGround(roll, pitch, yaw):
  return [[math.cos(pitch) * math.cos(yaw), 
        math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw),
        math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)],
       [math.cos(pitch) * math.sin(yaw),
        math.sin(roll) * math.sin(pitch) * math.sin(yaw) + math.cos(roll) * math.cos(yaw),
        math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw)],
       [-math.sin(pitch),
        math.sin(roll) * math.cos(pitch),
        math.cos(roll) * math.cos(pitch)]]

def RtoPutter(roll, pitch, yaw):
  return zip(*RtoGround(roll, pitch, yaw))


def verletDisplacement(a, dt):
  x = [0.0]
  v = [0.0]
  for i in range(1, len(a)):
    v_half = v[i-1] + 0.5*a[i-1]*dt
    x.append(x[i-1] + v_half*dt)
    v.append(v_half + 0.5*a[i]*dt)
  return x, v


def interpolateDisplacement(a, dt):
  x = [0.0]
  v = [0.0]
  for i in range(1, len(a)):
    v1 = v[i-1] + ((a[i] - a[i-1])/4 + a[i-1]) * (dt / 4)
    v2 = v1 + ((a[i] - a[i-1])/2 + a[i-1]) * (dt / 4)
    v3 = v2 + (3*(a[i] - a[i-1])/4 + a[i-1]) * (dt / 4)
    v.append(v3 + a[i] * (dt / 4))

    x1 = x[-1] + (v[-1] * (dt/4)) + (0.5 * ((a[i] - a[i-1])/4 + a[i-1]) * (dt/4)**2)
    x2 = x1 + (v1 * (dt/4)) + (0.5 * ((a[i] - a[i-1])/2 + a[i-1]) * (dt/4)**2)
    x3 = x2 + (v2 * (dt/4)) + (0.5 * (3*(a[i] - a[i-1])/4 + a[i-1]) * (dt/4)**2)
    x.append(x3 + (v3 * (dt/4)) + (0.5 * a[i] * (dt/4)**2))
  return x, v

'''
def rk4Displacement(x, v, a1, a2, dt):
  x1 = x
  v1 = v
  a1 = a

  x2 = x + 0.5*v1*dt
  v2 = v + 0.5*a1*dt
  a2 = 
 ''' 
  

def roll(accelX, accelY, accelZ):
  if accelX**2 + accelZ**2 == 0:
    print "would be divide by 0"
    return (math.atan(4.0 * 9.80665) * 180.0) / PI
  return (math.atan(accelY / math.sqrt(accelX**2 + accelZ**2)) * 180.0) / PI
  #return (math.atan2(-accelX, accelZ) * 180.0) / PI


def pitch(accelX, accelY, accelZ):
  #return (math.atan(accelY / (accelY**2 + accelZ**2)) * 180.0) / PI
  return (math.atan2(-accelX, accelZ) * 180.0) / PI


def yaw(roll, pitch, magX, magY, magZ):
  roll = math.radians(roll)
  pitch = math.radians(pitch)

  x = magX * math.cos(pitch) + magZ * math.sin(pitch)
  y = magX * math.sin(roll) * math.sin(pitch) + magY * math.cos(roll) - magZ * math.sin(roll) * math.cos(pitch)

  #x = magX * math.cos(pitch) + magY * math.sin(pitch) * math.sin(roll)
  #y = magY * math.cos(roll) + magZ * math.sin(roll)
  return (math.atan2(y, x) * 180.0) / PI


def displacement(d, v, a, dt):
  # displacement
	d1 = v*dt
	d2 = 0.5 * a * dt**2
	d = d1 + d2

	# velocity
	v1 = v
	v2 = a*dt
	v = v1 + v2

	return d,v


# Accelerometer conversion
def convertData(rawX, rawY, rawZ, calibration):
  data = lambda v: tosigned(v) / calibration 
  xyz = [data(rawX), data(rawY), data(rawZ)]
  mag = (xyz[0]**2 + xyz[1]**2 + xyz[2]**2)**0.5
  return (xyz, mag)


def getAngle(gyroData, dt):
    return -gyroData * dt


def rotateAcceleration(accel, angles):
  #angles: 0:roll, 1:pitch, 2:yaw
  #accel: 0:x, 1:y, 2:z
  gravity = rotateGravity(angles)
  accel[0] -= gravity[0]
  accel[1] -= gravity[1]
  accel[2] -= gravity[2]
  rotAccel = [0.0, 0.0, 0.0]
  roll = math.radians(angles[0])
  pitch = math.radians(angles[1])
  yaw = math.radians(angles[2])
  R = RtoGround(roll, pitch, yaw)
  for i in range(3):
    for j in range(3):
      rotAccel[i] += R[i][j] * accel[j]

  return rotAccel


def rotateGravity(angles):
  #gravityOffset = 0.00063
  gravity = [0.0,0.0,9.80665] #- gravityOffset]
  rotGravity = [0, 0, 0]
  roll = math.radians(angles[0])
  pitch = math.radians(angles[1])
  yaw = math.radians(angles[2])
  R = RtoPutter(roll, pitch, yaw)
  for i in range(3):
    for j in range(3):
      rotGravity[i] += R[i][j] * gravity[j]
  return rotGravity


def allanVariance(data, frequency, binSize, dt):
  numBins = int(math.floor(len(data) / binSize))
  bin_avg = []
  for i in range(0, numBins):
    start = i*binSize
    bin_avg.append(np.mean(data[start:start + binSize]))
  return (dt * binSize, np.std(bin_avg))



def create3plots(timeX, plotData, titleStr, xlabelStr, ylabelStr): # NOTE: I took out f_plotData
    # -> returns a figure with 3 subplots
    # NOTE: matplotlib.pyplot.show() [or plot.show(), as adjusted by import]
    #   must be called to render the plot

    # Parameters:
        # timeX     - must be a list of timestamps
        # plotData  - must be in the form of [xdataArray, ydataArray, zdataArray]

    # Example Usage:
        # > accelPlotData = [axData, ayData, azData]
        # > plotTitle = 'Acceleration Data'
        # > ylabel = r'Acceleration ($m/s^2$)'
        # > xlabel = r'Time ($s$)'
        # > figAccel = create3plots(timestamp, accelPlotData, plotTitle, xlabel, ylabel)

    # - configure the axes sizes
    ymin0 = min(plotData[0])
    ymin1 = min(plotData[1])
    ymin2 = min(plotData[2])

    ymax0 = max(plotData[0])
    ymax1 = max(plotData[1])
    ymax2 = max(plotData[2])

    ymax = max([ymax0,ymax1,ymax2])
    ymax = ymax + .015*ymax
    ymin = min([ymin0,ymin1,ymin2])
    ymin = ymin - (-.015)*ymin
    xmax = timeX[-1] 
    xmin = 0
    axesRange = [xmin, xmax, ymin, ymax]

    # - configure the axis ticks
    yticks = 10 # number of ticks
    xticks = 6
    scalarForXEndTicks = 1.0/(xticks)+(1.0/(10*xticks))
    scalarForYEndTicks = 1.0/(yticks)+(1.0/(10*yticks))
    xTicks = ((xmax + (scalarForXEndTicks)*xmax)/xticks)
    yTicks = ( (ymax - ymin) + scalarForYEndTicks*(ymax - ymin) )/yticks
    xTicks = float("{0:.2f}".format(xTicks) )
    yTicks = float("{0:.4f}".format(yTicks) )

        # prevent division by zero
    if xTicks == 0.00:
        xTicks = 0.01
    if yTicks == 0.00:
        yTicks = 0.01


    # declare the figure ---------------
    fig = plot.figure()
    fig.suptitle(titleStr, fontsize=14, fontweight='bold')


    # left subplot ---------------
    ax = fig.add_subplot(131)
    fig.subplots_adjust(top=0.85)
    ax.set_title(r'$X$')
    ax.set_ylabel(ylabelStr)


    ax.set_xticks( np.arange(xmin,xmax,xTicks) ) 
    ax.set_yticks( np.arange(ymin,ymax,yTicks) )

    ax.plot(timeX, plotData[0], color='blue')
    #ax.plot(timeX, f_plotData[0], color='black')

    ax.axis(axesRange)
    ax.grid(True)


    # middle subplot ------------------
    ax2 = fig.add_subplot(132)
    fig.subplots_adjust(top=0.85)
    ax2.set_title(r'$Y$')
    
    ax2.set_xlabel(xlabelStr)

    ax2.set_xticks( np.arange(xmin,xmax,xTicks) ) 
    ax2.set_yticks( np.arange(ymin,ymax,yTicks) )

    ax2.plot(timeX, plotData[1], color='red')
    #ax2.plot(timeX, f_plotData[1], color='black')

    ax2.axis(axesRange)
    ax2.grid(True)
    ax2.set_yticklabels([])


    # right subplot ---------------------
    ax3 = fig.add_subplot(133)
    fig.subplots_adjust(top=0.85)
    ax3.set_title(r'$Z$')

    ax3.set_xticks( np.arange(xmin,xmax,xTicks) ) 
    ax3.set_yticks( np.arange(ymin,ymax,yTicks) )

    ax3.plot(timeX, plotData[2], color='green')
    #ax3.plot(timeX, f_plotData[2], color='black')

    ax3.axis(axesRange)
    ax3.grid(True)
    ax3.set_yticklabels([])

    return fig

def getVeloAndDispLists(aList,timeX):
  # input a one dimensional list of acceleration data and one dimensional list of the timestamp
  # -> returns the corresponding velocity and displacement arrays
  vList = [0.0]
  dList = [0.0]
  for i in (range(len(aList)-1)):
    dPoint,vPoint = displacement(dList[-1], vList[-1], aList[i], timeX[i+1]-timeX[i])
    dList.append(dList[-1]+dPoint)
    vList.append(vPoint)
  return dList, vList



