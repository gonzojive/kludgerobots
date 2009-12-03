import roslib; roslib.load_manifest('hw3')
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
import math

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from roslib.rostime import Duration

LINE_WIDTH = .02
POINT_WIDTH = .09

class Visualizer:
    def __init__(self):
        self.pub = rospy.Publisher("visualization_marker", Marker)
        self.pubArray = rospy.Publisher("visualization_marker_array", MarkerArray)
        self.posePub = rospy.Publisher("particlecloud", PoseArray)
        self.idCounter = 400
        self.namedIdCounter = 1
        self.nameToId = {}
        self.nameToColor = {} # color is stored as a triple of (r g b)
        self.arrowIDMax = -1
        
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

    def otherStuff(self):
        print "did stuff"

    def makeMarker(self, markerType=Marker.LINE_LIST, name=None, color=None):
        # set the ID from a name
        the_id = None
        if name:
            the_id = self.getIdForName(name)

        marker = Marker()
        marker.header.frame_id = "/map";
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

        scale = LINE_WIDTH if markerType != Marker.POINTS else POINT_WIDTH
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

    def vizConnectedPoints(self, points, name=None, color=None):
        marker = self.makeMarker(markerType=Marker.LINE_STRIP, name=name, color=color)
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                             [vector3d(pt) for pt in points])        
        self.pub.publish(marker)

    def vizPoints(self, points, name=None, color=None):
        marker = self.makeMarker(markerType=Marker.POINTS, name=name, color=color)
        marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                             map(vector3d, points))        
        self.pub.publish(marker)

    def deleteArrow(self, idNum):
        #rospy.loginfo("Deleting arrows")
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.ns = "arrows"
        marker.action = Marker.DELETE
        marker.type = Marker.ARROW
        marker.id = idNum
        self.pub.publish(marker)

    def deleteArrows(self):
        for i in range(self.arrowIDMax+1):
            self.deleteArrow(i)

    def vizArrow(self, start, theta, size = [1.0, 1.0, 1.0], idNum = None, color = None, alpha = None):
        marker = Marker()   # create an empty Marker
        marker.header.frame_id = "/map"  # marker source frame
        marker.header.stamp = rospy.Time.now()  # timestamp - I think ros automatically adds these
        marker.ns = "arrows"    # namespace - might as well make it specific
        if idNum:   # if the caller wants to specify id numbers so the arrows last until overwritten
            marker.id = idNum
        else:   # otherwise, just give it a default number
            marker.id = self.idCounter
            self.idCounter += 1
        if marker.id > self.arrowIDMax:
            self.arrowIDMax = marker.id
        marker.type = Marker.ARROW  # arrow type marker
        marker.action = Marker.ADD  # adding a new marker
        # assign the marker starting position
        marker.pose.position.x = start[0]
        marker.pose.position.y = start[1]
        marker.pose.position.z = 0
        # create a quaternion by theta (from params) about the x axis (not sure why x works, but it does)
        quat = tf.transformations.quaternion_about_axis(theta, [0, 0, 1])
        #rospy.loginfo("Quaternion: %0.2f %0.2f %0.2f %0.2f", quat[0], quat[1], quat[2], quat[3])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
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
        marker.lifetime = rospy.Duration(0) # will live forever
        self.pub.publish(marker)

    def vizArrows(self, absoluteVectors, size = [.02, .02, .02], idNum = None, color = None, alpha = None):
        color = color
        def arrow(start, end, color=color):
            marker = Marker()   # create an empty Marker
            marker.header.frame_id = "/map"  # marker source frame
            #marker.header.stamp = rospy.Time()  # timestamp - I think ros automatically adds these
            marker.ns = "arrows"    # namespace - might as well make it specific
            if idNum:   # if the caller wants to specify id numbers so the arrows last until overwritten
                marker.id = idNum
            else:   # otherwise, just give it a default number
                marker.id = self.idCounter
                self.idCounter += 1
            if marker.id > self.arrowIDMax:
                self.arrowIDMax = marker.id
            marker.type = Marker.ARROW  # arrow type marker
            marker.action = Marker.ADD  # adding a new marker
            # assign the marker starting position
            marker.points = map (lambda pt : Point(x = pt[0], y = pt[1], z = pt[2]),
                                 [vector3d(start), vector3d(end)])
            # scale the marker if necessary
            marker.scale.x = size[0]
            marker.scale.y = size[1]
            marker.scale.z = size[2]
            # assign the marker color
            c = color
            if not c:
                color = [1.0, 1.0, 1.0] # default to a white arrow
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = alpha or 1.0   # if alpha is defined, use that, otherwise use 1.0
                marker.lifetime = rospy.Duration(0) # will live forever
            return marker
        markers = [arrow(start,end) for [start, end] in absoluteVectors]
        marr = MarkerArray()
        marr.markers = markers
        self.pubArray.publish(marr)

    def vizPoseArray(self, poses):
        poseArray = PoseArray()
        poseArray.header.frame_id = "/map"
        for p in poses:
            poseArray.poses.append(Pose())
            poseArray.poses[-1].position.x = p.x
            poseArray.poses[-1].position.y = p.y
            poseArray.poses[-1].position.z = 0
            quat = tf.transformations.quaternion_about_axis(p.theta, [0, 0, 1])
            poseArray.poses[-1].orientation.x = quat[0]
            poseArray.poses[-1].orientation.y = quat[1]
            poseArray.poses[-1].orientation.z = quat[2]
            poseArray.poses[-1].orientation.w = quat[3]
        self.posePub.publish(poseArray)

