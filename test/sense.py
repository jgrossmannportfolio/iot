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
delta_t = .12 # in seconds (preset to 120ms)
    # total duration
endtime = .61 # in seconds



# Global functions/hex and byte conversions
tosigned = lambda n: float(n-0x10000) if n>0x7fff else float(n)
tosignedbyte = lambda n: float(n-0x100) if n>0x7f else float(n)

# Global functions/retrieve start time --------------------------------------
current_milli_time = lambda: int(round(time.time() * 1000))
start_time = current_milli_time()
elapsed_time = lambda: float ("{0:.2f}".format( (float(current_milli_time()) - float(start_time))/1000 ) ) 


# Global lists
    # x
vx = 0.0
ax = 0.0
dx = 0.0
    # y
vy = 0.0
ay = 0.0
dy = 0.0
    # z
vz = 0.0
az = 0.0
dz = 0.0

xframes = [0.0]
yframes = [0.0]
zframes = [0.0]
xyzframes = []
xyzframes.append([0.0,0.0,0.0])

timestamp = []
iters = 0.0


# Get the frame, aka displacement from the origin, give the acceleration
def getFrame(d,v,a,accelData,frame):

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
	frame.append(float ("{0:.2f}".format(frame[-1] + d) ) )

	return d,v,a,frame




# Wiced sense class
class wicedsense:


  def pushToCloud(self, frames):
    #print frames[1]
    connection = httplib.HTTPSConnection("api.parse.com", 443)
    connection.connect()
    connection.request('PUT', '/1/classes/Putt/12fz4AHTDK', json.dumps({
       "frames": frames
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
    self.con.logfile = sys.stdout
    self.con.expect('\[LE\]>', timeout=600)
    print "Preparing to connect. You might need to press the side button..."
    self.con.sendline('connect')
    # test for success of connect
    self.con.expect('Connection successful.*\[LE\]>')
    print "Connection successful"

    self.cb = {}

    # Data from sensors
    self.time = []
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


    # Notification handle = 0x002b 
  def notification_loop( self ):
    global delta_t
    global endtime
    global timestamp
    global start_time
    global iters
    global xyzframes

    total = math.ceil( endtime/delta_t )

    iters = 0
    while total > iters:
      try:
        pnum = self.con.expect('Notification handle = .*? \r', timeout=4)
        print
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
          #print "after callback"
          pass
        else:
          print "TIMEOUT!!"
          pass
      else:
        print "else statement"

    # OUTPUT xframes TO SCREEN ------------------
    print
    print "xyzframes (frame indicates the x displacement in meters at a given time):"
    for x in range(0,len(xyzframes)): print xyzframes[x]
    #print xyzframes
    print
    print "total duration (in s) "
    print total*delta_t
    print
    print "total samples"
    print int (total) + 1 # add one for time 0
    print


    if (total + 1) != len(xyzframes):
      print "Length of frames does not match anticipated number of total samples"
      pass
    else:
      self.pushToCloud(xyzframes)
    

  def register_cb( self, handle, fn ):
    self.cb[handle]=fn
    return


  def dataCallback(self, v):
      global delta_t
      global endtime
      global timestamp
      global start_time
      global iters
    
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
	

      timestamp.append( elapsed_time() )
      print
      print "Python timestamp: " + str(timestamp[-1])
      print "Real time: " + str( iters*delta_t )

      bytelen = len(v)
    # Make sure 18 (?) bytes are received
    #if(v[0] == 3):
    #  print "CORRECT HANDLE"
      # =========================
      # GET TIMESTAMP
      # ==========================
      

      vx1 = int( str(v[2]*256 + v[1]) )
      vy1 = int( str(v[4]*256 + v[3]) )
      vz1 = int( str(v[6]*256 + v[5]) )
      gx1 = int( str(v[8]*256 + v[7]) )
      gy1 = int( str(v[10]*256 + v[9]) )
      gz1 = int( str(v[12]*256 + v[11]) )
      print " "
      #print "gx: " + str(gx1)
      #print "gy: " + str(gy1)
      #print "gz: " + str(gz1)
      #print "vx: " + str(vx1)
      #print "vy: " + str(vy1)
      #print "vz: " + str(vz1)
      #for x in range(0,19): print v[x]
      (Gxyz, Gmag) = self.convertData(gx1, gy1, gz1, 100.0)
      #(Axyz, Amag) = self.convertData(vx,vy,vz, (86.0/(9.80665 * 3779.53)))
      (Axyz, Amag) = self.convertData( vx1,vy1,vz1, 8192.0)#(8192.0/9.80665) )
      #print "accelx"
      #print Axyz[0]
      #self.accel.append(Axyz)
      #self.gyro.append(Gxyz)

      # ======================
      # GET DISPLACEMENT FRAMES
      # ======================
      
      dx,vx,ax,xframes = getFrame(dx,vx,ax,Axyz[0],xframes)
      dy,vy,ay,yframes = getFrame(dy,vy,ay,Axyz[1],yframes)
      dz,vz,az,zframes = getFrame(dz,vz,az,Axyz[2],zframes)

      xyzframes.append( [xframes[-1],yframes[-1],zframes[-1]] )

      #print "Axyz: " + str(Axyz)
      #print "Amag: " + str(Amag)
      #print "Gxyz: " + str(Gxyz)
      #print "Gmag: " + str(Gmag)
      return


class SensorCallbacks:
  data = {}
  def __init__(self,addr):
    self.data['addr'] = addr


# main function USAGE python sense.py <mac address>
def main():



  bluetooth_adr = sys.argv[1]
  #data['addr'] = bluetooth_adr
  if len(sys.argv) > 2:
      datalog = open(sys.argv[2], 'w+')

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
    start_time = current_milli_time()
    timestamp.append( elapsed_time() )
    print 
    print "Python timestamp (in s): " + str(timestamp[-1])
    print "Real time (in s): " + str( iters*delta_t )
    print
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





