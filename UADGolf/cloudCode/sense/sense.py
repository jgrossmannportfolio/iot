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



#hcitool lescan

import pexpect
import sys
import time
import httplib
import json




tosigned = lambda n: float(n-0x10000) if n>0x7fff else float(n)
tosignedbyte = lambda n: float(n-0x100) if n>0x7f else float(n)

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

	# Calibration vars
    self.xCal = [0,0,0]
    self.yCal = [0,0,0]
    self.zCal = [0,0,0]
	
    return

  def calibration (self, x, y, z):
    self.xCal[0] += x
    self.xCal[1] = min(self.xCal[1],x)
    self.xCal[2] = max(self.xCal[2],x)

    self.yCal[0] += y
    self.yCal[1] = min(self.yCal[1],y)
    self.yCal[2] = max(self.yCal[2],y)    
    
    self.zCal[0] += z
    self.zCal[1] = min(self.zCal[1],z)
    self.zCal[2] = max(self.zCal[2],z)


     

  # Function to write a value to a particular handle  
  def char_write_cmd( self, handle, value ):
    cmd = 'char-write-req 0x%02x 0%x' % (handle, value)
    #print cmd
    self.con.sendline( cmd )
    return

  # Funcrion to read from a handle
  def char_read_hnd(self, handle):
    self.con.sendline('char-read-hnd 0x%02x' % handle)
    self.con.expect('descriptor: .*? \r')
    after = self.con.after
    rval = after.split()[1:]
    #print rval
    # decode data obtained from the sensor tag
    self.processdata(rval)

  def processdata(self,data):

    # This is to decode and get sensor data from the packets we received
    # please refer to the broadcom data packet format document to understand the structure of the packet
    data_decode = ''.join(data)
    mask = int(data_decode[0:2],16)
    print "processing data"
    if(mask==0x0b):
      self.accelerometer = [int((data_decode[4:6] + data_decode[2:4]),16)]
      #self.accelerometer.append(int((data_decode[4:6] + data_decode[2:4]),16))
      self.accelerometer.append(int((data_decode[8:10] + data_decode[6:8]),16))
      self.accelerometer.append(int((data_decode[12:14] + data_decode[10:12]),16))
      self.gyroscope = [int((data_decode[16:18] + data_decode[14:16]),16)]
      #self.gyroscope.append(int((data_decode[16:18] + data_decode[14:16]),16))
      self.gyroscope.append(int((data_decode[20:22] + data_decode[18:20]),16))
      self.gyroscope.append(int((data_decode[24:26] + data_decode[22:24]),16))
      self.magnetometer = [int((data_decode[28:30] + data_decode[26:28]),16)]
      #self.magnetometer.append(int((data_decode[28:30] + data_decode[26:28]),16))
      self.magnetometer.append(int((data_decode[32:34] + data_decode[30:32]),16))
    else:
      self.humidity = int((data_decode[4:6] + data_decode[2:4]),16)
      self.pressure = int((data_decode[4:6] + data_decode[2:4]),16)/10
      self.temperature = int((data_decode[4:6] + data_decode[2:4]),16)/10

    # Notification handle = 0x002b 
  def notification_loop( self ):
    iterations = 25
    while True:
      try:
	    #print "in notification loop"
        if(len(self.time) >= iterations):
          print
          break
        '''
          print
          print "CALIBRATIONS:"
          self.xCal[0] /= iterations
          print self.xCal
          self.yCal[0] /= iterations
          print self.yCal
          self.zCal[0] /= iterations
          print self.zCal
          

        [15.4533, -20.0, 54.0]
        [-5.1469, -49.0, 38.0]
        [8210.8074, 0, 8280.0]

        '''

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
    frames = self.processData()
    self.pushToCloud(frames)
    

  def register_cb( self, handle, fn ):
    self.cb[handle]=fn
    return

  def dataCallback(self, v):
    bytelen = len(v)
    # Make sure 18 (?) bytes are received
    if(v[0] == 3):
      vx = int( str(v[2]*256 + v[1]) )
      vy = int( str(v[4]*256 + v[3]) )
      vz = int( str(v[6]*256 + v[5]) )
      gx = int( str(v[8]*256 + v[7]) )
      gy = int( str(v[10]*256 + v[9]) )
      gz = int( str(v[12]*256 + v[11]) )
      print " "
      print "gx: " + str(gx)
      print "gy: " + str(gy)
      print "gz: " + str(gz)
      print "vx: " + str(vx)
      print "vy: " + str(vy)
      print "vz: " + str(vz)
      #for x in range(0,19): print v[x]
      (Gxyz, Gmag) = self.convertData(gx, gy, gz, 100.0)
      (Axyz, Amag) = self.convertData(vx,vy,vz, 8192.0/9.80665) #(86.0/(9.80665 * 3779.53)))
      self.accel.append(Axyz)
      self.gyro.append(Gxyz)
      print "Axyz: " + str(Axyz)
      print "Amag: " + str(Amag)
      print "Gxyz: " + str(Gxyz)
      print "Gmag: " + str(Gmag)

      self.calibration(Axyz[0], Axyz[1], Axyz[2])

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

    print bluetooth_adr
    tag = wicedsense(bluetooth_adr)
    cbs = SensorCallbacks(bluetooth_adr)

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
