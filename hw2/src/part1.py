#!/usr/bin/env python
import roslib; roslib.load_manifest('hw2')
import rospy
import tf
from robotPosition import *
import viz
import pose
import os
import move
import math
import motionModel
import particleFilter
import threading
import mapmodel
import geometry_msgs
import nav_msgs

from sensor_msgs.msg import LaserScan

class Part1():
    def __init__(self):
        self._position = RobotPosition()
        self._odoListener = None
        self._visualizer = viz.Visualizer()
        self._move = move.MoveFromKeyboard(self._visualizer)
        self._pFilter = None
        # I have no idea what good error values are
        self._motionErr = motionModel.MotionErrorModel(.005, 0.0005, .01, .005, 0.0005, .005)
        #self._motionErr = None  # test only, assumes no error
        #self._motionErr = motionModel.MotionErrorModel(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)

    def robotPosition(self):
        return self._position

    def odoListener(self):
        return self._odoListener

    def initFilter(self):
        # will eventually want to pass in an error model
        self._pFilter = particleFilter.ParticleFilter(self.robotPosition().position(), self._visualizer, None, self._motionErr, self.odoListener(), pose.Pose(34, 46, 0), False)
        self._pFilter.updateMapTf()    # initialize the best guess and sent it to tf
        self._pFilter.displayPoses()
        self._pFilter.start()   # threading function, calls our overloaded run() function and begins execution
        #self._pFilter.poseSet.printPoses()

    def initSubscriptions(self):
        # subscribe to transformation updates
        self._odoListener = tf.TransformListener() # listen to tf
        rospy.loginfo("Subscribed to odometry frames")

    # perform an update when we receive new info from the odometer or whatever
    def update(self, trans, rot):
        self.robotPosition().odomReadingNew(trans, rot)
        self._move.publishNextMovement()
        self._pFilter.receiveOdom(self.robotPosition().position())
        self._pFilter.mapModel.broadcast()
        
    def initNode(self):
        rospy.init_node('kludge2_1')
        rospy.loginfo('"KLUDGE hw2.1" node is awake')

        self.initSubscriptions()
        
        rate = rospy.Rate(10.0) # 10 Hz

        # initialize the robot global compass / odometry
        while not self.robotPosition().initialized:
            try:
                (trans, rot) = self.odoListener().lookupTransform('/odom', '/base_link', rospy.Time(0))
                self.robotPosition().initialized = True
            except (tf.LookupException, tf.ConnectivityException):
                continue
        
        self.robotPosition().resetOdom(trans, rot)
        self.initFilter()

        # while we are not shutdown by the ROS, keep updating
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.odoListener().lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            
            self.update(trans, rot)
            pass    # give another thread a chance to run
            rate.sleep()


if __name__ == '__main__':
    try:
        app = Part1()
        app.initNode()
    except rospy.ROSInterruptException:
        # try to remove our arrows so we don't have to restart roscore
        # doesn't work :( I'll just put a long timer on the arrows
        # actually, the IDs are the same, so you can just restart the program
        # app._visualizer.deleteArrows()
        pass
