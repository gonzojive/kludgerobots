#!/usr/bin/env python
import roslib; roslib.load_manifest('hw1')
import rospy
import tf
from math import *
from line import *
from util import *
from vector import *
from lineviz import *
from robotPosition import *
from move import *
from compass import *

# import cv	# doesn't recognize cv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Part2:
    def __init__(self):
        self._position = RobotPosition()
        self._compass = Compass()
        self._laserInterpreter = LaserInterpreter(self.robotPosition())
        self._odoListener = None
        self._moveToGoal = MoveToGoal(self.robotPosition(), self.laserInterpreter())

    def robotPosition(self):
        return self._position

    def compass(self):
        return self._compass

    def laserInterpreter(self):
        return self._laserInterpreter

    def odoListener(self):
        return self._odoListener

    def moveToGoalAgent(self):
        return self._moveToGoal

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
        self.moveToGoalAgent().publishNextMovement()
        
    def initNode(self):
        rospy.init_node('kludge1_2')
        rospy.loginfo('"KLUDGE hw1.2" node is awake')

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

        self.moveToGoalAgent().setGoal( self.robotPosition().globalToLocal([4.0, 0.0]))

        # while we are not shutdown by the ROS, keep updating
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.odoListener().lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            #    rospy.loginfo("Odometry: (%0.2f, %0.2f) at %0.2f", trans[0], trans[1], rot[2])
            self.update(trans, rot)
            #    cmd.send()
            rate.sleep()


if __name__ == '__main__':
  try:
      app = Part2()
      app.initNode()
  except rospy.ROSInterruptException:
    pass

