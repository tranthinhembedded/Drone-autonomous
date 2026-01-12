import numpy as np

class KalmanFilter:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0

        self.P = np.zeros((2, 2))

    def getAngle(self, newAngle, newRate, dt):
        self.rate = newRate - self.bias
        self.angle += dt * self.rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure
        K = np.zeros(2)
        K[0] = self.P[0][0] / S
        K[1] = self.P[1][0] / S

        y = newAngle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

    def setAngle(self, angle):
        self.angle = angle

    def getRate(self):
        return self.rate

    def setQAngle(self, Q_angle):
        self.Q_angle = Q_angle

    def setQBias(self, Q_bias):
        self.Q_bias = Q_bias

    def setRMeasure(self, R_measure):
        self.R_measure = R_measure

    def getQAngle(self):
        return self.Q_angle

    def getQBias(self):
        return self.Q_bias

    def getRMeasure(self):
        return self.R_measure
