#!/usr/bin/env python
import roslib; roslib.load_manifest('hw2')
import rospy
import tf
from robotPosition import *
import viz
import pose
import os
import move

from sensor_msgs.msg import LaserScan

class Part1:
    def __init__(self):
        self._position = RobotPosition()
        self._laserInterpreter = LaserInterpreter()
        self._odoListener = None
        self._visualizer = viz.Visualizer()
        self._move = move.MoveFromKeyboard(self._visualizer)
        self._poses = pose.PoseSet(self._visualizer, numPoses = 64) # will eventually want to pass in an error model here as well

    def robotPosition(self):
        return self._position

    def laserInterpreter(self):
        return self._laserInterpreter

    def odoListener(self):
        return self._odoListener


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

        #self.moveToGoalAgent().setGoal( self.robotPosition().globalToLocal([10.0, 0.0]))

        # while we are not shutdown by the ROS, keep updating
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.odoListener().lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                continue
            
            self.update(trans, rot)
            rate.sleep()


if __name__ == '__main__':
#    savedTTYState = os.popen('stty -g', 'r').read()
    try:
#        os.system('stty raw')
        app = Part1()
        app.initNode()
    except rospy.ROSInterruptException:
        pass
#    finally:
#        os.system('stty ' + savedTTYState)
