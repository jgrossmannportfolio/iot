#!/usr/bin/python
import time;  # This is required to include time module.
import math;

# INPUTS -------------------------------------------------------------------

# create an interval array to indicate when the sensor data is received (in s)
	# (of course this will be nonlinear in practice)
interval = [0,.1,.1,.1,.1,.1,.1,.1,.1,.1,.1]

# Create an acceleration list to be simulated (in m/s^2)
	# (Make sure that the accel list length is larger than the interval list)
#accelx = [0,10,10,10,10,10,-10,-10,-10,-10,-10]
#accelx = [0,10,10,10,10,10,10,10,10,10,10]
#accelx = [0,10,-10,10,-10,10,-10,10,-10,10,-10]
#accelx = [0,-10,10,-10,10,-10,10,-10,10,-10,10]
#accelx = [0,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10]
#accelx = [0,10,-10,0,0,0,0,0,0,0,0]
#accelx = [0,-10,10,0,0,0,0,0,0,0,0]
accelx = [0,-10,10,15,0,0,0,0,0,0,0]

# --------------------------------------------------------------------------
# calculate endtime based on the interval list
endtime = 0.0
for x in range(0,len(interval)): endtime += interval[x]


# Global functions/retrieve start time --------------------------------------
current_milli_time = lambda: int(round(time.time() * 1000))
start_time = current_milli_time()
elapsed_time = lambda: float ("{0:.2f}".format( (float(current_milli_time()) - float(start_time))/1000 ) ) 


# Get the frame, aka displacement from the origin, give the acceleration
def getFrame(d,v,a,accelData,frame):

	global timestamp
	global delta_t

	# acceleration
	a.append( accelData )
	
	# displacement
	d1 = v[-1]*(delta_t)
	d2 = (0.5)*(a[-1])*((delta_t)**2)
	d.append( d1 + d2 )

	# velocity
	v1 = v[-1]
	v2 = a[-1]*delta_t
	v.append( v1 + v2 )

	# frame
	frame.append( float ("{0:.2f}".format(frame[-1] + d[-1]) ) )

	return d,v,a,frame



# Preallocating lists/Iteration 0 ------------------------------------------
timestamp = []
timestamp.append( elapsed_time() )
print 
print "timestamp (in s)"
print timestamp[0]

vx = []
vx.append( 0.0 )

ax = []
ax.append( float(accelx[0]) )

dx = []
dx.append( 0.0 )

xframes = []
xframes.append( 0.0 )



# MAIN loop ================================================================
i = 1 	# begin loop with iteration 1
while i < int( math.floor(endtime*10.0)) +2:
	time.sleep(interval[i])
	timestamp.append( elapsed_time() )
	delta_t = timestamp[-1]-timestamp[-2]

	print(timestamp[i])
	dx,vx,ax,xframes = getFrame(dx,vx,ax,accelx[i],xframes)
	
	i += 1


# OUTPUT xframes TO SCREEN -------------------------------------------------
print
print "xframes (frame indicates the x displacement in meters at a given time):"
for x in range(0,len(xframes)): print xframes[x]
print
#print "timestamp"
#for x in range(0,len(timestamp)): print timestamp[x]



