import rospy
import tf
import pose
import threading


class MapManager:
    def __init__(self):
        self.mapData = None
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.currentTf = [[0, 0, 0], [0, 0, 0, 1], rospy.Time.now()]
        self.tempTf = self.currentTf[:]
#        self.tfLock = threading.lock()  # needed in case the main thread and the filter access it at the same time

    def updateMapToOdomTf(self, pose, odom, time = None):
        self.tempTf[0][0] = pose.x - odom[0]
        self.tempTf[0][1] = pose.y - odom[1]
        self.tempTf[2] = time or rospy.Time.now()
        self.tempTf[1] = tf.transformations.quaternion_from_euler(0, 0, pose.theta-odom[2])
#        self.tfLock.acquire()   # <--- grab the lock --->
        self.currentTf = self.tempTf[:]  # deep copy
#        self.tfLock.release()   # <--- release the lock --->

    def broadcast(self):
#        self.tfLock.acquire()   # <--- grab the lock --->
        self.tfBroadcaster.sendTransform(self.currentTf[0], self.currentTf[1], self.currentTf[2], "odom", "map")
#        self.tfLock.release()   # <--- release the lock --->
#        rospy.loginfo("Sent broadcast")
