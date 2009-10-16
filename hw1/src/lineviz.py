import roslib; roslib.load_manifest('hw1')
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
from vector import *

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from roslib.rostime import Duration

LINE_WIDTH = .02
POINT_WIDTH = .04

class LocalMapVisualizer:
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

    
    def makeMarker(self, type=Marker.LINE_LIST, name=None, color=None):
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
        marker.type = type;
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
        marker = self.makeMarker(type=Marker.LINE_LIST, name=name, color=color)
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                             [vector3d(start), vector3d(end)])        
        self.pub.publish(marker)

    def vizSegments(self, points, name=None, color=None):
        marker = self.makeMarker(type=Marker.LINE_STRIP, name=name, color=color)
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                             map(vector3d, points))        
        self.pub.publish(marker)

    def vizPoints(self, points, the_id=None):
        marker = Marker()
        marker.header.frame_id = "/base_laser";
        marker.ns = "basic_shapes";
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
        marker.lifetime.secs = 700.100
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1]), points)
        self.pub.publish(marker)

