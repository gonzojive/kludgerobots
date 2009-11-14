import rospy
import roslib

from geometry_msgs.msg

class Goal:
    def __init__(goalX, goalY, goalTheta):
        self.x = goalX
        self.y = goalY
        self.theta = goalTheta
