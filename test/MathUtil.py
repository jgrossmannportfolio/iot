# Math utility functions
import math

PI = 3.1415926536

# Global functions/hex and byte conversions
tosigned = lambda n: float(n-0x10000) if n>0x7fff else float(n)
tosignedbyte = lambda n: float(n-0x100) if n>0x7f else float(n)

def roll(accelY, accelZ):
  return (math.atan2(accelY, accelZ) * 180.0) / PI


def pitch(accelX, accelY, accelZ):
  return (math.atan2(accelX, math.sqrt(accelY**2 + accelZ**2)) * 180.0) / PI


def yaw(roll, pitch, magX, magY, magZ):
  x = magX * math.cos(pitch) + magY * math.sin(pitch) * math.sin(roll)
  y = magY * math.cos(roll) + magZ * math.sin(roll)
  return (math.atan2(-y / x) * 180.0) / PI


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
    return gyroData * dt

