import math
from math import *

class fakeOdom:
    MAX_VEL = 0.9
    MAX_ANG_VEL = math.pi
    MAX_ACCEL = 0.5
    MAX_ANG_ACCEL = math.pi / 1.6
    SIMULATION_DT = .3
    SIMULATION_T = 4.0
    # constructs a velocity from provided robot controls (forward velocity and angular velocity)
    #
    # forward velocity is a scalar value in meters per second and is the velocity of the robot in
    #   the positive X direction
    #
    # angularVelocity is a scalar value in radians per second and it corresponds to the rotation
    #   of the robot about its center of rotation
    def __init__(self):
        self.fwdVel = 0
        self.angVel = 0
        self.currentFwdVel = 0
        self.currentAngVel = 0
        self.x = 0
        self.y = 0
        self.theta = 0
    
    def getPosition(self):
        return [[self.x,self.y],self.theta]
 

    def simulate(self,fwdVel,angVel):
        
        theta = 0.0
        [x, y] = [0.0, 0.0]
        dt = 0.1
         
        newTheta = theta + dt * angVel  # calculate theta after dt
        halfwayTheta = (newTheta + theta)/2.0  # find the halfway theta
        theta = newTheta
    
        self.x += fwdVel*cos(halfwayTheta)*dt  # estimate the x position after dt
        self.y += fwdVel*sin(halfwayTheta)*dt  # estimate the y position after dt

        self.theta += theta
      
              

