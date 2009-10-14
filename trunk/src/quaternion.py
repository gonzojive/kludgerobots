import math
import numpy

from vector import *
from util import *

def qconjugate(quat):
    (x, y, z, w) = quat
    return (-x, -y, -z, w)

def qToAxisAngle(quat):
    (x, y, z, w) = quat
    angle = math.acos(w) * 2.0
    if angle < .01 and angle > -.01:
        return [ [0.0, 0.0, 1.0], 0.0]
    else:
        axis = vector_scale( (x, y, z), 1.0 / math.sin(angle / 2.0))
        return [ axis, angle ]

def quatToAngleAboutPositiveZ(quat):
     [ axis, angle ] = qToAxisAngle(quat)
     if axis[2] < -.99999 and axis[2] > -1.00111:
         return normalizeAngle360(-1.0 * angle)
     elif axis[2] > .999 and axis[2] < 1.00111:
         return normalizeAngle360(angle)
     else:
         raise "Invalid  z axis! %s" % axis
                                  

# these functions might not work
def qmultiply(q1, q2):
    (x, y, z, w) =  (q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
                     q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2],
                     q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0],
                     q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2])
    return (x, y, z, w)

def rotateVectorWithQuaternion(vector, quat):
    qvector = (vector[0], vector[1], vector[2], 0)
    qresult = qmultiply(qvector, qconjugate(quat))
    (x, y, z, w) =  qresult
    return (x, y, z)

