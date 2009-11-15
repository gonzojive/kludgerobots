#!/usr/bin/env python
import roslib; roslib.load_manifest('hw3')
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
import laser
import goal

from sensor_msgs.msg import LaserScan

class Part2():
    def __init__(self):
        self._position = RobotPosition()
        self._goals = goal.GoalSet()
        self._laser = laser.Laser()
        self._odoListener = None
        self._visualizer = viz.Visualizer()
        self._move = move.MoveFromKeyboard(self._visualizer, self._goals)
        self._pFilter = None
        self.mapModel = None
        # I have no idea what good error values are
        self._motionErr = motionModel.MotionErrorModel(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
        self.initialPose = pose.Pose(34.0, 46.0, 0.0)
        #self._motionErr = None  # test only, assumes no error


    def robotPosition(self):
        return self._position

    def laserInterpreter(self):
        return self._laser

    def odoListener(self):
        return self._odoListener

    def initFilter(self):
        self._pFilter = particleFilter.ParticleFilter(self.robotPosition().position(),
                                                      self._visualizer,
                                                      self.laserInterpreter(),
                                                      self._motionErr,
                                                      self.odoListener(),
                                                      # initial pose in the MAP frame
                                                      self.initialPose,
                                                      self.mapModel)
        self._pFilter.updateMapTf()    # initialize the best guess and sent it to tf
        self._pFilter.displayPoses()
        self._pFilter.start()   # threading function, calls our overloaded run() function and begins execution
        #self._pFilter.poseSet.printPoses()

    def initSubscriptions(self):
        # subscribe to laser readings
        def laserCallback(reading):
            self.laserInterpreter().laserReadingNew(reading)

        rospy.Subscriber("laser", LaserScan, laserCallback) # listen to "laser"
        rospy.loginfo("Subscribed to laser readings")

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
        rospy.init_node('kludge3')
        rospy.loginfo('"KLUDGE hw3" node is awake')

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

        self.mapModel = mapmodel.MapModel(self.initialPose)
        while not self.mapModel.initializedp():
            pass

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
        app = Part2()
        app.initNode()
    except rospy.ROSInterruptException:
        # try to remove our arrows so we don't have to restart roscore
        # doesn't work :( I'll just put a long timer on the arrows
        # actually, the IDs are the same, so you can just restart the program
        # app._visualizer.deleteArrows()
        pass