'''
Adapted from code snippets taken from http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/


Kalman filter implementation
Updates a 1 dimensional angle of rotation based on stream of data:
    newAngle: pitch, roll, yaw based on accelerometer data
    newRate: angular velocity data from gyroscope (1-d deg/s)
    dt: timestep between two consecutive data polls
'''


class Kalman:
    #static bias and error variables
    Q_angle = .0002 #low: don't trust acceleration that much, smoother to accel angle
                 #high: trust acceleration data, follows accel closer with more noise
    Q_bias = .001 #amound of drift do to gyro
    R_measure = 0.0004

    angle = 0.0
    bias = 0.0

    P = [ [0 for j in range(2)] for i in range(2) ]  #initialize size of matrix 2x2
    P[0][0] = 0.0 #may have to change the matrix, assuming bias is 0 and that we know the initial starting angle
    P[0][1] = 0.0
    P[1][0] = 0.0
    P[1][1] = 0.0
 
    def __init__(self, startAngle):
        self.angle = startAngle
# The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    def updateAngle(self, newAngle, newRate, dt):

        #predict new angle based on prior data
        rate = newRate - self.bias  #angular velocity with bias offset
        self.angle += dt * rate    #integrated angle based on angular velocity and timestep

        #Update estimation error covariance - Project the error covariance ahead
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        #Calculate Kalman gain
        S = self.P[0][0] + self.R_measure  #Estimate error
        K = [0 for i in range(2)] #kalman gain 2x1 vector
        K[0] = self.P[0][0] / S
        K[1] = self.P[1][0] / S

        # Calculate angle and bias - Update estimate with measurement zk (newAngle)
        y = newAngle - self.angle

        self.angle += K[0] * y
        self.bias += K[1] * y

        #Calculate estimation error covariance - Update the error covariance
        temp00 = self.P[0][0]
        temp01 = self.P[0][1]

        self.P[0][0] -= K[0] * temp00
        self.P[0][1] -= K[0] * temp01
        self.P[1][0] -= K[1] * temp00
        self.P[1][1] -= K[1] * temp01

        return self.angle

