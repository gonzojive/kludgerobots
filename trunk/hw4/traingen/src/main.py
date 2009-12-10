#!/usr/bin/env python
import roslib; roslib.load_manifest('traingen')
import rospy
import tf
import os
import math
import threading
import geometry_msgs
from sensor_msgs.msg import LaserScan
import nav_msgs
import wx

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D, Vector3, PointStamped
from nav_msgs.msg import Odometry
from stage.msg import Puck

class Traingen():
    # returns the puck centers relative to the laser frame

    def transformPoint(self, point, base_frame, target_frame):
        p = PointStamped()
        p.header.stamp = rospy.Time()
        p.header.frame_id = base_frame
        p.point.x = point[0]
        p.point.y = point[1]
        p.point.z = 0
        p = self.tfListener.transformPoint(target_frame, p)
        return [p.point.x, p.point.y]
        
    def get_puckCenters(self):
        pucks = [self.pucksDict[puckId] for puckId in self.pucksDict]
        def puckCenter(puck):
            return self.transformPoint([puck.x, puck.y], "map", "base_laser_ground_truth")
            puckAsPoint = PointStamped()
            puckAsPoint.header.stamp = rospy.Time()
            puckAsPoint.header.frame_id = "map"
            puckAsPoint.point.x = puck.x
            puckAsPoint.point.y = puck.y
            puckAsPoint.point.z = 0
            transformedPoint = self.tfListener.transformPoint("base_laser_ground_truth", puckAsPoint)
            return [transformedPoint.point.x, transformedPoint.point.y]
        return [puckCenter(puck) for puck in pucks]

    def generateTrainingPair(self):
        """
        Returns [input training string, label string]
        """
        rospy.loginfo( "generating training pair." )
        robotPosition = self.transformPoint([0,0], "base_laser_ground_truth", "map")
        output = "%f %f %f\n" % (robotPosition[0], robotPosition[1], 90.0)

        scan = self.laserScan
        puckCenters = self.puckCenters
        r2d = 180.0/3.14159
        output += "%i %f %f %f\n" % (len(scan.ranges), r2d*scan.angle_min, r2d*scan.angle_max, r2d*scan.angle_increment)
        for r in scan.ranges:
            output += "%f\n" % r

        label = "%d\n" % len(puckCenters)
        for pt in puckCenters:
            label += "%f %f\n" % (pt[0], pt[1])

        rospy.loginfo( "finished generating training pair." )
        return [output, label]

    puckCenters = property(get_puckCenters)
    

    def initSubscriptions(self):
        #self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        # subscribe to laser readings
        #def laserCallback(reading):
        #    self.laserInterpreter().laserReadingNew(reading)
        
        #rospy.Subscriber("laser", LaserScan, laserCallback) # listen to "laser"
        #rospy.loginfo("Subscribed to laser readings")
        tfBroadcaster = tf.TransformBroadcaster()
        tfListener = tf.TransformListener()

        self.tfListener = tfListener
        
        def truePositionCallback(trueOdom):
            #rospy.loginfo("Got true odometry. %s" % trueOdom)
            trans = [-trueOdom.pose.pose.position.y, trueOdom.pose.pose.position.x, trueOdom.pose.pose.position.z]
            orient = trueOdom.pose.pose.orientation
            rot = [orient.x, orient.y, orient.z, orient.w]
            tfBroadcaster.sendTransform(trans, rot, rospy.Time(), "base_laser_ground_truth", "map")

        self.pucksDict = {}
        def cylinderCallback(puck):
            self.pucksDict[puck.puckId] = puck

        def laserCallback(scan):
            self.laserScan = scan
            
        # subscribe to transformation updates
        rospy.Subscriber("pucks", Puck, cylinderCallback)
        rospy.Subscriber("base_pose_ground_truth", Odometry, truePositionCallback)
        rospy.Subscriber("laser", LaserScan, laserCallback)
        #self._odoListener = tf.TransformListener() # listen to tf
        #rospy.loginfo("Subscribed to odometry frames")
        
    def initNode(self):
        rospy.init_node('final_traingen')
        rospy.loginfo('"final_traingen" node is awake')

        self.initSubscriptions()
        
        rate = rospy.Rate(10.0) # 10 Hz

        # initialize the robot global compass / odometry
        # while we are not shutdown by the ROS, keep updating
        while not rospy.is_shutdown():
            #send commands                
            #if self._gradients.initializationDone:  
            
            rate.sleep()    
            
        rospy.loginfo("Exiting Kludge")

class TrainingGenerationFrame(wx.Frame):
    def __init__(self, parent, id, title, traingen):
        wx.Frame.__init__(self, parent, id, title, size = (200, 200))
        panel = wx.Panel(self, -1)
        box = wx.BoxSizer(wx.HORIZONTAL)
        self.outputButton = wx.Button(self, -1, 'Output Training File', (190, 185), (160, -1))
        #self.outputValue = wx.StaticText(panel, -1, "output1", (45, 25), style=wx.ALIGN_CENTRE)
        box.Add(self.outputButton)
        #box.Add(self.outputValue)
        panel.SetSizer(box)
        def theOnClick(event):
            self.onClick(event)
        #panel.Bind(wx.EVT_BUTTON, theOnClick)
        self.Bind(wx.EVT_BUTTON, theOnClick)

        rospy.loginfo( "set up click events." )

        self.traingen = traingen
        self.outputNumber = 0

        #self.Add(cutoffSlider)

        
    def outputTrainingPair(self, programInput, label):
        rospy.loginfo( "writing output." )
        fInputName = None
        while not fInputName or os.path.exists(fInputName):
            self.outputNumber += 1
            fInputName = "/home/red/ros/pkgs/kludge/hw4/traingen/training-pairs/input%i.dat" % self.outputNumber
            
        fInput = open(fInputName, 'w')
        fLabel = open("/home/red/ros/pkgs/kludge/hw4/traingen/training-pairs/label%i.dat" % self.outputNumber, 'w')
        fInput.write(programInput)
        fLabel.write(label)
        fInput.close()
        fLabel.close()
        

    def onClick(self, event):
        rospy.loginfo( "wants output." )
        [output, label] = self.traingen.generateTrainingPair()
        self.outputTrainingPair(output, label)
        rospy.loginfo( "outputted training pair." )

if __name__ == '__main__':
    try:
                
        app = Traingen()
        class GUI(threading.Thread):
            def run(self):
                wxApp = wx.App()
                f = TrainingGenerationFrame( None, -1, "CS225B final TrainGen", app)
                f.Show()
                wxApp.MainLoop()

        GUI().start()
        app.initNode()
    except rospy.ROSInterruptException:
        # try to remove our arrows so we don't have to restart roscore
        # doesn't work :( I'll just put a long timer on the arrows
        # actually, the IDs are the same, so you can just restart the program
        # app._visualizer.deleteArrows()
        pass
