import math


# MotionErrorModel
#   contains information about our estimated error model for the robot
class MotionErrorModel:
    def __init__(self, a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0, a6 = 0):
        # Theta1 error depends on Theta1 and Delta
        self.sigTheta1 = a1
        self.sigDeltaOnTheta1 = a2
        # Delta error depends only on Delta
        self.sigDelta = a3
        # Theta2 error depends on Theta1, Delta, and Theta2
        self.sigTheta1OnTheta2 = a4
        self.sigDeltaOnTheta2 = a5
        self.sigTheta2 = a6
    def theta1Variance(self, motion):
        return abs(motion.theta1*self.sigTheta1) + abs(motion.delta*self.sigDeltaOnTheta1)
    def deltaVariance(self, motion):
        return abs(motion.delta*self.sigDelta)
    def theta2Variance(self, motion):
        return abs(motion.theta1*self.sigTheta1OnTheta2) + abs(motion.delta*self.sigDeltaOnTheta2) + abs(motion.theta2*self.sigTheta2)

# Motion
#   contains information about a single motion: initial turn, displacement, final turn
class Motion:
    def __init__(self, theta1, delta, theta2):
        self.theta1 = theta1
        self.delta = delta
        self.theta2 = theta2
    def toStr(self):
        return "theta1: %0.2f  delta: %0.2f  theta2: %0.2f" % (self.theta1, self.delta, self.theta2)


# motionModelToOdom(): converts from a motion model to odometry
# parameters:
#   model   --  a Motion instance
def motionModelToOdom(model):
    dx = model.delta * math.cos(model.theta1)
    dy = model.delta * math.sin(model.theta1)
    da = model.theta1 + model.theta2
    return [dx, dy, da]

# odomToMotionModel(): converts initial and final odom readings into our motion model variables
# parameters:
#   odomInitial --  [x, y, angle] list of the initial odometry
#   odomFinal   --  [x, y, angle] list of the final odometry
def odomToMotionModel(odomInitial, odomFinal):
    # calculate (final - initial)
    [dx, dy, da] = [odomFinal[0]-odomInitial[0], odomFinal[1]-odomInitial[1], odomFinal[2]-odomInitial[2]]
    theta1 = math.atan2(dy, dx) - odomInitial[2]  # initial turn
    delta = math.sqrt(dx*dx + dy*dy)    # distance traveled
    theta2 = da - theta1   # final turn
    return Motion(theta1, delta, theta2)
