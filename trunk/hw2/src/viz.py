import rospy
import tf
import visualization_msgs
import visualization_msgs.msg
import roslib.message
import roslib.rostime
import geometry_msgs.msg
import roslib.msg
import std_msgs.msg
import random
import tf.transformations
from vector import *

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from roslib.rostime import Duration

LINE_WIDTH = .02
POINT_WIDTH = .04

class Visualizer:
    def __init__(self):
        self.pub = rospy.Publisher("visualization_marker", Marker)
        self.idCounter = 400
        self.namedIdCounter = 1
        self.nameToId = {}
        self.nameToColor = {} # color is stored as a triple of (r g b)
        
    def getIdForName(self, name):
        if name not in self.nameToId:
             self.nameToId[name] = self.namedIdCounter
             self.namedIdCounter = self.namedIdCounter + 1
        return self.nameToId[name]
    
    def setMarkerColor(self, marker, name, explicit_color=None):
        def randColor():
            return [random.random() for x in range(0, 3)]
        color = explicit_color or randColor()
        if name:
            if name not in self.nameToColor:
                self.nameToColor[name] = explicit_color or randColor()
            color = self.nameToColor[name]
        [r, g, b] = color
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

    
    def makeMarker(self, markerType=Marker.LINE_LIST, name=None, color=None):
        # set the ID from a name
        the_id = None
        if name:
            the_id = self.getIdForName(name)

        marker = Marker()
        marker.header.frame_id = "/base_laser";
        marker.ns = "basic_shapes";
        if the_id:
            marker.id = the_id
        else:
            marker.id = self.idCounter;
        self.idCounter = self.idCounter + 1
        marker.type = markerType;
        marker.action = Marker.ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        scale = LINE_WIDTH
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        
        self.setMarkerColor(marker, name, color)
        
        marker.lifetime.secs = .100
        return marker

    def vizSegment(self, start, end, name=None, color=None):
        marker = self.makeMarker(markerType=Marker.LINE_LIST, name=name, color=color)
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                             [vector3d(start), vector3d(end)])        
        self.pub.publish(marker)

    def vizSegments(self, points, name=None, color=None):
        marker = self.makeMarker(markerType=Marker.LINE_STRIP, name=name, color=color)
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                             map(vector3d, points))        
        self.pub.publish(marker)

    def vizArrow(self, start, theta, size = [1.0, 1.0, 1.0], idNum = None, color = None, alpha = None):
        marker = Marker()   # create an empty Marker
        marker.header.frame_id = "/base_laser"  # marker source frame
        marker.header.stamp = rospy.Time()  # timestamp
        marker.ns = "arrows"    # namespace - might as well make it specific
        if idNum:   # if the caller wants to specify id numbers so the arrows last until overwritten
            marker.id = idNum
        else:   # otherwise, just give it a default number
            marker.id = self.idCounter
            self.idCounter += 1
        marker.type = Marker.ARROW  # arrow type marker
        marker.action = Marker.ADD  # adding a new marker
        # assign the marker starting position
        marker.pose.position.x = start[0]
        marker.pose.position.y = start[1]
        marker.pose.position.z = 0
        # create a quaternion by theta (from params) about the x axis (not sure why x works, but it does)
        quat = tf.transformations.quaternion_about_axis(theta, [1, 0, 0])
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        # scale the marker if necessary
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        # assign the marker color
        if not color:
            color = [1.0, 1.0, 1.0] # default to a white arrow
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = alpha or 1.0   # if alpha is defined, use that, otherwise use 1.0
        marker.lifetime = rospy.Duration(1)
        self.pub.publish(marker)

    def vizPoints(self, points, the_id=None):
        marker = Marker()
        marker.header.frame_id = "/base_laser"
        marker.ns = "basic_shapes"
        if the_id:
            marker.id = the_id
        else:
            marker.id = self.idCounter;
        self.idCounter = self.idCounter + 1
        marker.type = Marker.POINTS;
        marker.action = Marker.ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = POINT_WIDTH;
        marker.scale.y = POINT_WIDTH;
        marker.scale.z = POINT_WIDTH;
        marker.color.r = 1.0;
        marker.color.g = .40;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime.secs = .100
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1]), points)
        self.pub.publish(marker)

