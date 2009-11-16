import math

def vector3d(v):
    if len(v) == 2:
        return [v[0], v[1], 0.0]
    else:
        return v
    

def cross3d(v, w):
    x = v[1]*w[2] - v[2]*w[1]
    y = v[2]*w[0] - v[0]*w[2]
    z = v[0]*w[1] - v[1]*w[0]
    return (x, y, z)

def cross2d(v, w):
    return cross3d([v[0], v[1], 0], [w[0], w[1], 0])

def cross(v,w):
    if len(v) == 2:
        return cross2d(v,w)
    else:
        return cross3d(v,w)

def vector_minus(b, a):
    a_to_b = map(lambda acoord, bcoord: bcoord - acoord, a, b)
    return a_to_b

def vector_add(b, a):
    a_plus_b = map(lambda acoord, bcoord: bcoord + acoord, a, b)
    return a_plus_b

def vector_scale(v, s):
    return map(lambda x: x * s, v)

# v is a tuple representing a 3d vector
def vector_length(v):
    sum = 0
    for x in v:
        sum = sum + x*x
    return math.sqrt(sum)

def vector_length_squared(v):
    sum = 0
    for x in v:
        sum = sum + x*x
    return sum

def normalize(v):
    l = vector_length(v)
    return map(lambda x : x / l, v)

def vector_normalize(v):
    l = vector_length(v)
    return map(lambda x : x / l, v)

def vector_dot(v,w):
    return sum(map( lambda a,b: a * b, v, w))

def vector_angle_normalized(v, w):
    dot_result =  vector_dot(v,w)
    
    try:
	result = math.acos(dot_result)
 	return result
    except:
        # math domain error
	return 0

#oddly, this assumes normalized angles
def vector_angle(v, w):
    return vector_angle_normalized(v, w)

def vector_angle_general(v, w):
    return vector_angle_normalized(normalize(v),normalize(w))

def vector_angle_signed(v, w):
    #return math.atan2(w[1] - v[1], w[0] - v[0])
    
    ang = math.atan2(v[1],v[0]) - math.atan2(w[1],w[0])
    if ang > math.pi:
        ang = 2.0 * math.pi - ang
    # we are in a right-handed system so return the opposite sign
    return ang * -1.0

    #z = [
    #signed_angle = atan2(  N * ( V1 x V2 ), V1 * V2  );
    # where * is dot product and x is cross product
    # N is the normal to the polygon
    # ALL vectors: N, V1, V2 must be normalized

    #sign = (vector_dot(
    #    c = cross(y,z);
    #     angleyz = sign(dot(x,c))*atan2(norm(c),dot(y,z));
    #3angleyz = sign(dot(x,c))*atan2(norm(c),dot(y,z));

# rotates the vector theta degrees counterclockwise about the positive z axis
def vector_rotate_2d(v, theta_about_z):
    theta = theta_about_z
    [x, y] = v
    return [x * math.cos(theta) - y * math.sin(theta), x * math.sin(theta) + y * math.cos(theta)]

def lineDistanceToPoint(pt, line_origin, line_trajectory):
    w = vector_minus(pt, line_origin)
    traj_cross_w = cross(line_trajectory, w)
    return vector_length(traj_cross_w) / vector_length(line_trajectory)


