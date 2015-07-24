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

import MathUtil
import kalman

# Global functions/retrieve start time --------------------------------------
current_milli_time = lambda: int(round(time.time() * 1000))
start_time = current_milli_time()
elapsed_time = lambda: float ("{0:.4f}".format( (float(current_milli_time()) - float(start_time))/1000 ) )


# Wiced sense class
class wicedsense:

  # frequency of data
  delta_t = .0125 # in seconds (preset to 12.5 ms)
  # total duration
  endtime = 4.0 # in seconds
  garbageiterations = 10

  inittime = 0 
  starttimer = 0 # wait for garbageiterations before the timer starts

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

  magnitude = []
  timestamp = []

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

  # Init function to connect to wiced sense using mac address
  # Blue tooth address is obtained from blescan.py 
  def __init__( self, bluetooth_adr, calibrate ):
    self.con = pexpect.spawn('gatttool -b ' + bluetooth_adr + ' --interactive')
    #self.con.logfile = sys.stdout
    self.con.expect('\[LE\]>', timeout=600)
    print "Preparing to connect. You might need to press the side button..."
    self.con.sendline('connect')
    # test for success of connect
    self.con.expect('Connection successful.*\[LE\]>')
    print "Connection successful"
    self.calibrate = calibrate
    self.cb = {}
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
    if(self.calibrate == False):
      calText = self.readFromFile("calibration.txt")
      calArray = calText.split(",")
      self.gyroCal = calArray[0:3]
      self.accelCal = calArray[3:]
      print self.gyroCal
      print self.accelCal

    # LOOP UNTIL TIME EXPIRES
    if(self.calibrate):
      total = math.ceil(30.0 / self.delta_t)
    else:
      total = math.ceil( self.endtime/self.delta_t )

    iters = 0    
    while total > iters:
      try:
        pnum = self.con.expect('Notification handle = .*? \r', timeout=4)
        
        # wait for the BT to settle -> wait for program to cycle through garbage iterations
        if self.starttimer == 1:
          iters += 1

      except pexpect.TIMEOUT:
        print "TIMEOUT exception!"
        break

      try:
        if pnum==0:
          after = self.con.after
          hxstr = after.split()[3:]
          handle = long(float.fromhex(hxstr[0]))
          self.cb[handle]([long(float.fromhex(n)) for n in hxstr[2:]])
        else:
          print "pnum not equal to 0"

      except Exception,e:
        print str(e)
        '''else:
          print "TIMEOUT!!"
          pass'''
      


    # After the while loop has broken...
    gyroAvg = [0, 0, 0]
    accelAvg = [0, 0, 0]
    if(self.calibrate == True):
      calText = ""
      gyroAvg[0] = fsum(self.gyroX) / len(self.gyroX)
      gyroAvg[1] = fsum(self.gyroY) / len(self.gyroY)
      gyroAvg[2] = fsum(self.gyroZ) / len(self.gyroZ)
      accelAvg[0] = fsum(self.ax) / len(self.ax)
      accelAvg[1] = fsum(self.ay) / len(self.ay)
      accelAvg[2] = fsum(self.az) / len(self.az)
      calText += str(0 - gyroAvg[0]) + "," + str(0 - gyroAvg[1]) + "," + str(0 - gyroAvg[2]) + ","
      calText += str(0 - accelAvg[0]) + "," + str(0 - accelAvg[1]) + "," + str(8192 - accelAvg[2])
      print "writing to file"
      self.writeToFile("calibration.txt", calText)
    
    else:
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
      for x in range(len(self.magnitude)):
        print self.magnitude[x]
        if self.magnitude[x] > thresh:
          print "PUTTING................."
          axnew = self.ax[x:]
          aynew = self.ay[x:]
          aznew = self.az[x:]
          gyroXnew = self.gyroX[x:]
          gyroYnew = self.gyroY[x:]
          gyroZnew = self.gyroZ[x:]
          break

      # ========================
      # GET DISPLACEMENT FRAMES
      # ========================

      roll = []
      pitch = []
      yaw = []

      xframes = [MathUtil.displacement(self.dx,self.vx,axnew[0], self.delta_t)[0]]
      yframes = [MathUtil.displacement(self.dy,self.vy,aynew[0], self.delta_t)[0]]
      zframes = [MathUtil.displacement(self.dz,self.vz,aznew[0], self.delta_t)[0]]
      roll.append(MathUtil.roll(aynew[0], aznew[0]))
      pitch.append(MathUtil.pitch(axnew[0], aynew[0], aznew[0]))
      xyzframes = [xframes[-1], yframes[-1], zframes[-1]] 
      for x in range(1, len(axnew)):
        
        roll.append(MathUtil.roll(aynew[x], aznew[x]))
        pitch.append(MathUtil.pitch(axnew[x], aynew[x], aznew[x]))
        self.dx,self.vx = MathUtil.displacement(self.dx,self.vx,axnew[x], self.delta_t)
        xframes.append( float(xframes[-1] + self.dx)  )
        self.dy,self.vy = MathUtil.displacement(self.dy,self.vy,aynew[x], self.delta_t)
        yframes.append( float(yframes[-1] + self.dy)  )
        self.dz,self.vz = MathUtil.displacement(self.dz,self.vz,aznew[x], self.delta_t)
        zframes.append( float(zframes[-1] + self.dz)  )
        xyzframes.append( [xframes[-1], yframes[-1], zframes[-1]] )

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
      gyrodata = []
      kalmanX = kalman.Kalman(roll[0])
      kalmanY = kalman.Kalman(pitch[0])
      #kalmanZ = kalman.Kalman(gyroZnew[0])
      gyrodata.append([roll[0], pitch[0], MathUtil.getAngle(gyroZnew[0], self.delta_t)])
      for x in range(1, len(gyroXnew)):
        gyrodata.append([kalmanX.updateAngle(roll[x], gyroXnew[x], self.delta_t), 
                        kalmanX.updateAngle(pitch[x], gyroYnew[x], self.delta_t), 
                        MathUtil.getAngle(gyroZnew[x], self.delta_t) ])
        #gyrodata.append( [ MathUtil.getAngle(gyroXnew[x], self.delta_t), MathUtil.getAngle(gyroYnew[x], self.delta_t), MathUtil.getAngle(gyroZnew[x], self.delta_t) ] )

      # =======================
      # PUSH FRAMES TO PARSE
      # =======================
      self.pushToCloud(xyzframes, gyrodata)
        

  def register_cb( self, handle, fn ):
    self.cb[handle]=fn
    return


  def dataCallback(self, v):
    # clear first ten recordings because BT timing needs to settle
    # garbageiterations = 10
    if self.inittime > self.garbageiterations-1:

      if self.starttimer == 0:
        self.starttimer = 1
        start_time = current_milli_time()
        self.timestamp.append( 0.0 )
        #print "Python timestamp (in s): " + str(timestamp[-1])

      else: # the garbage iterations have passed
        self.timestamp.append( elapsed_time() )
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

        if(self.calibrate == True):
          (Gxyz, Gmag) = MathUtil.convertData(gx1, gy1, gz1, 1.0) # FS = 500dps
          (Axyz, Amag) = MathUtil.convertData( vx1,vy1,vz1, 1.0)#(8192.0/9.80665))
        else:
          (Gxyz, Gmag) = MathUtil.convertData(gx1 + int(float(self.gyroCal[0])), gy1 + int(float(self.gyroCal[1])), gz1 + int(float(self.gyroCal[2])), 1.0/.0175)
          (Axyz, Amag) = MathUtil.convertData(vx1 + int(float(self.accelCal[0])), vy1 + int(float(self.accelCal[1])), vz1 + int(float(self.accelCal[2])), 8192.0/9.80665)
                    
        print str(Gxyz[0]) +", "+str(Gxyz[1])+", "+str(Gxyz[2]) + ", " + str(Axyz[0]) + ", " + str(Axyz[1]) + ", " + str(Axyz[2]) 

        self.gyroX.append(Gxyz[0])
        self.gyroY.append(Gxyz[1])
        self.gyroZ.append(Gxyz[2])

        self.magnitude.append( Amag )

        self.ax.append( Axyz[0] )
        self.ay.append( Axyz[1] )
        self.az.append( Axyz[2] )
 
    else:
      self.inittime += 1  # increment 10 times before evaluating values      


# main function USAGE python sense.py <mac address>
def main():
  calibrate = False
  bluetooth_adr = sys.argv[1]
  
  if len(sys.argv) > 2:
    if(sys.argv[2] == "true" or sys.argv[2] == "True"):
      calibrate = True

 # while True:
  try:   
    print "[re]starting.."

    tag = wicedsense(bluetooth_adr, calibrate)

    tag.register_cb(0x2a,tag.dataCallback)
    tag.char_write_cmd(0x2b, 0x01)

    #tag.char_read_hnd(0x2a)
    tag.notification_loop()

  except Exception, e:
    print str(e)
    pass

if __name__ == "__main__":
  main()


