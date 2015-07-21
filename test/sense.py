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

import pexpect
import sys
import time
import httplib
import json

import math;



# Global functions/hex and byte conversions
tosigned = lambda n: float(n-0x10000) if n>0x7fff else float(n)
tosignedbyte = lambda n: float(n-0x100) if n>0x7f else float(n)

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




# Wiced sense class
class wicedsense:

  def pushToCloud(self, frames):
    print frames[0]
    connection = httplib.HTTPSConnection("api.parse.com", 443)
    connection.connect()
    connection.request('PUT', '/1/classes/Putt/12fz4AHTDK', json.dumps({
       "frames": frames
     }), {
       "X-Parse-Application-Id": "iAFEw9XwderX692l0DGIwoDDHcLTGMGtcBFbgMqb",
       "X-Parse-REST-API-Key": "I0xfoOS0nDqaHxfSBTgLNMuXGtsStl7zO0XZVDZX",
       "Content-Type": "application/json"
     }) 


  def processData(self):
    frames = []
    interval = 1.0 / 10.0
    t0 = self.time[0]
    t1 = self.time[1]
    i = 1
    vx = 0
    vy = 0
    vz = 0
    x = 0
    y = 0
    z = 0
    time = 0;
    print "processing data"
    print len(self.accel)
    while i < len(self.accel):
        t1 = self.time[i]
        if((t1 - t0) >= interval):
            time = interval
            interval = 1.0/10.0
        else:
            time = t1 - t0
            interval -= time

        x += vx*time + (0.5 * self.accel[i][0] * time**2)
        y += vy*time + (0.5 * self.accel[i][1] * time**2)
        z += vz*time + (0.5 * self.accel[i][2] * time**2)
        vx += self.accel[i][0] * time
        vy += self.accel[i][1] * time
        vz += self.accel[i][2] * time

        print "ax, ay, az: "+ str(self.accel[i][0]) + ", " + str(self.accel[i][1]) + ", " + str(self.accel[i][2])
        print "vx, vy, vz: "+ str(vx) + ", " + str(vy) + ", "+ str(vz)
        print "x, y, z: "+ str(x) + ", " + str(y) + ", " + str(z)
        print "time, interval: "+ str(time) + ", "+str(interval)
        if(interval == (1.0/10.0)):
            frames.append([x, y, z])
        else:
            i += 1
        t0 += time          

    return frames
            
        

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
    while True:
      try:
	    #print "in notification loop"
        if(len(self.time) >= 15):
          # OUTPUT xframes TO SCREEN ------------------
          print
          print "xframes (frame indicates the x displacement in meters at a given time):"
          for x in range(0,len(xframes)): print xframes[x]
          print
          break


        pnum = self.con.expect('Notification handle = .*? \r', timeout=4)
        self.time.append(time.time())
        #print "Printing in notification loop"
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

    # frames = self.processData()
    # self.pushToCloud(frames)
    

  def register_cb( self, handle, fn ):
    self.cb[handle]=fn
    return


  def dataCallback(self, v):
    global timestamp
    global delta_t
    global dx
    global vx
    global ax
    global xframes
    # =========================
    # GET TIMESTAMP
    # ==========================
    timestamp.append( elapsed_time() )
    delta_t = timestamp[-1] - timestamp[-2]
    print(timestamp[-1])
	
	
    bytelen = len(v)
    # Make sure 18 (?) bytes are received
    if(v[0] == 11):
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
      (Axyz, Amag) = self.convertData( vx1,vy1,vz1, (8192.0/9.80665) )
      print "accelx"
      print Axyz[0]
      #self.accel.append(Axyz)
      #self.gyro.append(Gxyz)

      # ======================
      # GET DISPLACEMENT
      # ======================
      
      dx,vx,ax,xframes = getFrame(dx,vx,ax,Axyz[0],xframes)
      
      print "Axyz: " + str(Axyz)
      print "Amag: " + str(Amag)
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
    global timestamp
    global dx
    global vx
    global ax
    global xframes
    global start_time
    start_time = current_milli_time()
    timestamp = []
    timestamp.append( elapsed_time() )
    print 
    print "timestamp (in s)"
    print timestamp[0]
    vx = []
    vx.append( 0.0 )
    ax = []
    ax.append( float(0.0) )
    dx = []
    dx.append( 0.0 )
    xframes = []
    xframes.append( 0.0 )
    # ----------------------------------------------------------------------------

    #print cbs.data['addr']
    #print "registering"
    tag.register_cb(0x2a,tag.dataCallback)
    tag.char_write_cmd(0x2b, 0x01)
    tag.char_write_cmd(0x31, 0x01)


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





