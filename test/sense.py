#!/usr/bin/env python
# Michael Saunby. April 2013
#
# Notes.
# pexpect uses regular expression so characters that have special meaning
# in regular expressions, e.g. [ and ] must be escaped with a backslash.
#
#   Copyright 2013 Michael Saunby
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

# MAC: 
#20:73:7A:15:13:DE

import pexpect
import sys
import time
import httplib
import json

import math
import operator



# Global inputs
    # frequency of BLE transmissions
delta_t = .0125 # in seconds (preset to 12.5 ms)
    # total duration
endtime = 4.0 # in seconds
garbageiterations = 10

# Global initialization triggers
inittime = 0 
starttimer = 0 # wait for garbageiterations before the timer starts



# Global functions/hex and byte conversions
tosigned = lambda n: float(n-0x10000) if n>0x7fff else float(n)
tosignedbyte = lambda n: float(n-0x100) if n>0x7f else float(n)

# Global functions/retrieve start time --------------------------------------
current_milli_time = lambda: int(round(time.time() * 1000))
start_time = current_milli_time()
elapsed_time = lambda: float ("{0:.4f}".format( (float(current_milli_time()) - float(start_time))/1000 ) ) 


# Global lists
    # x

calibrate = False
accelCal = [0.0, 0.0, 0.0]
gyroCal = [0.0, 0.0, 0.0]

vx = 0.0
ax = [0.0]
dx = 0.0
    # y
vy = 0.0
ay = [0.0]
dy = 0.0
    # z
vz = 0.0
az = [0.0]
dz = 0.0

gyroX = []
gyroY = []
gyroZ = []

xframes = [0.0]
yframes = [0.0]
zframes = [0.0]
xyzframes = []
magnitude = []

timestamp = []
iters = 0.0


# Get the frame, aka displacement from the origin, give the acceleration
def getFrame(d,v,accelData,frame):

	global delta_t
	
	# displacement
	d1 = v*(delta_t)
	d2 = (0.5)*(accelData)*((delta_t)**2)
	d = d1 + d2

	# velocity
	v1 = v
	v2 = accelData*delta_t
	v = v1 + v2

	# frame
	frame.append( float(frame[-1] + d)  )

	return d,v,frame




# Wiced sense class
class wicedsense:


  def pushToCloud(self, frames, gyrodata):
    #print frames[1]
    connection = httplib.HTTPSConnection("api.parse.com", 443)
    connection.connect()
    connection.request('PUT', '/1/classes/Putt/12fz4AHTDK', json.dumps({
       "frames": frames,
       "gyro": gyrodata
     }), {
       "X-Parse-Application-Id": "iAFEw9XwderX692l0DGIwoDDHcLTGMGtcBFbgMqb",
       "X-Parse-REST-API-Key": "I0xfoOS0nDqaHxfSBTgLNMuXGtsStl7zO0XZVDZX",
       "Content-Type": "application/json"
     }) 

  # Accelerometer conversion
  def convertData(self, rawX, rawY, rawZ, calibration):
    data = lambda v: tosigned(v) / calibration 
    xyz = [data(rawX), data(rawY), data(rawZ)]
    mag = (xyz[0]**2 + xyz[1]**2 + xyz[2]**2)**0.5
    return (xyz, mag)
    

  # Init function to connect to wiced sense using mac address
  # Blue tooth address is obtained from blescan.py 
  def __init__( self, bluetooth_adr ):
    self.con = pexpect.spawn('gatttool -b ' + bluetooth_adr + ' --interactive')
    #self.con.logfile = sys.stdout
    self.con.expect('\[LE\]>', timeout=600)
    print "Preparing to connect. You might need to press the side button..."
    self.con.sendline('connect')
    # test for success of connect
    self.con.expect('Connection successful.*\[LE\]>')
    print "Connection successful"

    self.cb = {}

    # Data from sensors
    self.accel = []
    self.gyro = []
    self.magnetometer=[]
    self.humidity = 0
    self.temperature = 0
    self.pressure = 0
    return

  # Function to write a value to a particular handle  
  def char_write_cmd( self, handle, value ):
    cmd = 'char-write-req 0x%02x 0%x' % (handle, value)
    #print cmd
    self.con.sendline( cmd )
    return

  def writeToFile(self, filename, text):
    file = open(filename, 'w')
    file.write(text)
    file.close

  def readFromFile(self, filename):
    file = open(filename, 'r')
    return file.read()

    # Notification handle = 0x002b 
  def notification_loop( self ):
    global delta_t
    global endtime
    global timestamp
    global start_time
    global iters
    global xyzframes

    global gyroCal
    global accelCal
    
    global dx
    global vx
    global ax

    global dy
    global vy
    global ay

    global dz
    global vz
    global az

    global xframes
    global yframes
    global zframes
    global xyzframes

    global magnitude
    global starttimer
    global garbageiterations

    global gyroX
    global gyroY
    global gyroZ
    global calibrate

    if(calibrate == False):
        calText = self.readFromFile("calibration.txt")
        calArray = calText.split(",")
        gyroCal = calArray[0:3]
        accelCal = calArray[3:]
        print gyroCal
        print accelCal

    # LOOP UNTIL TIME EXPIRES
    if(calibrate):
        total = math.ceil(30.0 / delta_t)
    else:
        total = math.ceil( endtime/delta_t )
    iters = 0    
    while total > iters:
      try:
        pnum = self.con.expect('Notification handle = .*? \r', timeout=4)
        
        # wait for the BT to settle -> wait for program to cycle through garbage iterations
        if starttimer == 1:
          iters += 1

      except pexpect.TIMEOUT:
        print "TIMEOUT exception!"
        break
      if pnum==0:
        #print "pnum = 0"
        after = self.con.after
        hxstr = after.split()[3:]
        handle = long(float.fromhex(hxstr[0]))
        if True:
	  try:
            self.cb[handle]([long(float.fromhex(n)) for n in hxstr[2:]])

          except Exception,e:
            print str(e)

          pass
        else:
          print "TIMEOUT!!"
          pass
      else:
        print 


    # After the while loop has broken...
    gyroAvg = [0, 0, 0]
    accelAvg = [0, 0, 0]
    if(calibrate == True):
        calText = ""
        gyroAvg[0] = sum(gyroX) / len(gyroX)
        gyroAvg[1] = sum(gyroY) / len(gyroY)
        gyroAvg[2] = sum(gyroZ) / len(gyroZ)
        accelAvg[0] = sum(ax) / len(ax)
        accelAvg[1] = sum(ay) / len(ay)
        accelAvg[2] = sum(az) / len(az)
        calText += str(0 - gyroAvg[0]) + "," + str(0 - gyroAvg[1]) + "," + str(0 - gyroAvg[2]) + ","
        calText += str(0 - accelAvg[0]) + "," + str(0 - accelAvg[1]) + "," + str(8192 - accelAvg[2])
        print "writing to file"
        self.writeToFile("calibration.txt", calText)
    
    else:

        for x in range(len(ax)):
            ax[:] = [i + float(accelCal[0]) for i in ax]
            ay[:] = [i + float(accelCal[1]) for i in ay]
            az[:] = [i + float(accelCal[2]) for i in az]
            gyroX[:] = [g + float(gyroCal[0]) for g in gyroX]
            gyroY[:] = [g + float(gyroCal[1]) for g in gyroY]
            gyroZ[:] = [g + float(gyroCal[2]) for g in gyroZ]

        # FILTER OUT INITIAL ACCELERATION VALUES --------------
        thresh = 9.9 # accel treshold must be exceeded to indicate putt has begun (m/s^2)
        axnew = []   # new acceleration list in the x direction
        aynew = []   # ... y direction
        aznew = []   # ... z direction
        gyroXnew = []
        gyroYnew = []
        gyroZnew = []
        print
        print "magnitude"
        for x in range(len(magnitude)):
          print magnitude[x]
          if magnitude[x] > thresh:
              print "PUTTING................."
              axnew = ax[x:]
              aynew = ay[x:]
              aznew = az[x:]
              gyroXnew = gyroX[x:]
              gyroYnew = gyroY[x:]
              gyroZnew = gyroZ[x:]
              break
        
        # ========================
        # GET DISPLACEMENT FRAMES
        # ========================
        for x in range(len(axnew)):
          dx,vx,xframes = getFrame(dx,vx,axnew[x],xframes)
          dy,vy,yframes = getFrame(dy,vy,aynew[x],yframes)
          dz,vz,zframes = getFrame(dz,vz,aznew[x],zframes)
          xyzframes.append( [xframes[-1],yframes[-1],zframes[-1]] )

        # OUTPUT TO SCREEN ------------------
        '''
        print "axnew:"
        for x in range(0,len(axnew)): print axnew[x]
        print

        print "axnew length"
        print len(axnew)
        print

        print "xyzframes (frame indicates the x displacement in meters at a given time):"
        for x in range(0,len(xyzframes)): print xyzframes[x]
        print
        
        print "total duration (in s) "
        print (float(len(axnew)))*delta_t
        print

        print "total samples"
        print int (len(axnew)) + 1 # add one for time 0
        print

        '''
        # gyro averages
        '''
        print
        print "GYRO averages (x,y,z):"
        print sum(gyroX)/len(gyroX)
        print sum(gyroY)/len(gyroY)
        print sum(gyroZ)/len(gyroZ)
        '''

        gyrodata = []
        for x in range(len(gyroXnew)):
            gyrodata.append( [ gyroXnew[x],gyroYnew[x],gyroZnew[x] ] )

        # =======================
        # PUSH FRAMES TO PARSE
        # =======================
        self.pushToCloud(xyzframes, gyrodata)
        

  def register_cb( self, handle, fn ):
    self.cb[handle]=fn
    return


  def dataCallback(self, v):
      global delta_t
      global endtime
      global timestamp
      global start_time
      global iters

      global ax
      global ay
      global az
      global inittime
      global garbageiterations
      global starttimer

      global gyroX
      global gyroY
      global gyroZ
    
      global calibrate
      global acccelCal
      global gyroCal

      # clear first ten recordings because BT timing needs to settle
      # garbageiterations = 10
      if inittime > garbageiterations-1:

        if starttimer == 0:
              starttimer = 1
              start_time = current_milli_time()
              timestamp.append( 0.0 )
              #print "Python timestamp (in s): " + str(timestamp[-1])

        else: # the garbage iterations have passed
              timestamp.append( elapsed_time() )
              #print "Python timestamp (in s): " + str(timestamp[-1])

              bytelen = len(v) # v is the handle data

            # Make sure bytes are received by the correct handle
            #if(v[0] == 3):

              vx1 = int( str(v[2]*256 + v[1]) )
              vy1 = int( str(v[4]*256 + v[3]) )
              vz1 = int( str(v[6]*256 + v[5]) )
              gx1 = int( str(v[8]*256 + v[7]) )
              gy1 = int( str(v[10]*256 + v[9]) )
              gz1 = int( str(v[12]*256 + v[11]) )
              #print "gx: " + str(gx1)
              #print "gy: " + str(gy1)
              #print "gz: " + str(gz1)
              #print "vx: " + str(vx1)
              #print "vy: " + str(vy1)
              #print "vz: " + str(vz1)
              #for x in range(0,19): print v[x]
              if(calibrate == True):
                  (Gxyz, Gmag) = self.convertData(gx1 + int(gyroCal[0]), gy1 + int(gyroCal[1]), gz1 + int(gyroCal[2]), 1.0)
                  (Axyz, Amag) = self.convertData(vx1 + int(accelCal[0]), vy1 + int(accelCal[1]), vz1 + int(accelCal[2]), 1.0)
              else:
                  (Gxyz, Gmag) = self.convertData(gx1, gy1, gz1, 1.0/.0175) # FS = 500dps
                  #(Axyz, Amag) = self.convertData(vx,vy,vz, (86.0/(9.80665 * 3779.53)))
                  (Axyz, Amag) = self.convertData( vx1,vy1,vz1, 8192.0/9.80665)#(8192.0/9.80665)
              #print "accelx"
              #print Axyz[0]
              #self.accel.append(Axyz)
              #self.gyro.append(Gxyz)
              #print "Gyro (x,y,z)"
              print str(Gxyz[0]) +", "+str(Gxyz[1])+", "+str(Gxyz[2]) + ", " + str(Axyz[0]) + ", " + str(Axyz[1]) + ", " + str(Axyz[2]) 

              gyroX.append(Gxyz[0])
              gyroY.append(Gxyz[1])
              gyroZ.append(Gxyz[2])

              # append magnitude of the acceleration
              magnitude.append( Amag )
              # append acceleration values
              ax.append( Axyz[0] )
              ay.append( Axyz[1] )
              az.append( Axyz[2] )

       
      else:
        inittime += 1  # increment 10 times before evaluating values      
      return


class SensorCallbacks:
  data = {}
  def __init__(self,addr):
    self.data['addr'] = addr


# main function USAGE python sense.py <mac address>
def main():
  global calibrate
  bluetooth_adr = sys.argv[1]
  #data['addr'] = bluetooth_adr
  
  if len(sys.argv) > 2:
    if(sys.argv[2] == "true" or sys.argv[2] == "True"):
      calibrate = True
      #datalog = open(sys.argv[2], 'w+')

 # while True:
  try:   
    print "[re]starting.."

    tag = wicedsense(bluetooth_adr)
    cbs = SensorCallbacks(bluetooth_adr)


    # Preallocating Global lists (Iteration 0) -----------------------------------
    global delta_t
    global endtime
    global timestamp
    global start_time
    global iters

    #timestamp for debugging
    #start_time = current_milli_time()
    #timestamp.append( elapsed_time() )


    # ----------------------------------------------------------------------------

    #print cbs.data['addr']
    #print "registering"
    tag.register_cb(0x2a,tag.dataCallback)
    tag.char_write_cmd(0x2b, 0x01)
    #tag.char_write_cmd(0x31, 0x01)



    #tag.char_write_cmd(0x2e, 0x0100)
    #tag.char_write_cmd(0x2b,0x01)
    # Read from handle 
    #tag.char_read_hnd(0x2a)
    #tag.frequency.append(time.time())
    tag.notification_loop()
    #print "after notification loop"
    i=1
    #while i<50:
    	# Print the data
    	#print ("Accel: ")
   	#print(tag.accelerometer) 
   	#print ("Gyro: ")
    	#print (tag.gyroscope)
        #print ("Magneto: ")
        #print (tag.magnetometer)
	#tag.char_write_cmd(0x2b,0x01)
	#tag.char_read_hnd(0x2a)
	#tag.frequency.append(time.time())
	#i = i + 1
        
    # @avi: Add final REST API code to push data to Parse Cloud
    #print (tag.frequency[-1] - tag.frequency[0])


  except Exception, e:
    print str(e)
    pass

if __name__ == "__main__":
    main()





