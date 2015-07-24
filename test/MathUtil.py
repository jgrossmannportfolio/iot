# Math utility functions

PI = 3.1415926536

def roll(accelY, accelZ):
  return (atan2(accelY, accelZ) * 180.0) / PI


def pitch(accelX, accelY, accelZ):
  return (atan2(accelX, sqrt(accelY**2, accelZ**2)) * 180.0) / PI


def yaw(roll, pitch, magX, magY, magZ):
  x = magX * cos(pitch) + magY * sin(pitch) * sin(roll)
  y = magY * cos(roll) + magZ * sin(roll)
  return (atan2(-y / x) * 180.0) / PI


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

