#!/usr/bin/env python
#ROS
import roslib; roslib.load_manifest('hw3')
import rospy
import tf

import nav_msgs
import nav_msgs.msg
from sensor_msgs.msg import LaserScan

# kludge
import util
import quaternion
import vector
import distgrid
import viz

class HW3:
    def initNode(self):
        rospy.init_node('kludge3_1')
        rospy.loginfo('"KLUDGE hw3.1" node is awake')

        rate = rospy.Rate(10.0) # 10 Hz
        v = viz.Visualizer()

        def mapCallback(occGrid):
            rospy.loginfo('KLUDGE got map data.  Building up the gradient grid.')
            grid = distgrid.computeAndVizGradientGrid(occGrid, v)
            
        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, mapCallback) # listen to "map"

        # while we are not shutdown by the ROS, keep updating
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        app = HW3()
        app.initNode()
    except rospy.ROSInterruptException:
        # try to remove our arrows so we don't have to restart roscore
        # doesn't work :( I'll just put a long timer on the arrows
        # actually, the IDs are the same, so you can just restart the program
        # app._visualizer.deleteArrows()
        pass
