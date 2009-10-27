import rospy
import tf
import pose
import threading

from nav_msgs.msg import OccupancyGrid


class ROSMap:
    def __init__(self):
        self.loadTime = None
        self.resolution = None
        self.width = 0
        self.height = 0
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]
        self.mapData = []

    def mapCallback(self, data):
        self.loadTime = data.info.map_load_time
        self.resolution = data.info.resolution
        self.width = data.info.width
        self.height = data.info.height
        self.position = [data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z]
        self.orientation = [data.info.origin.orientation.x, data.info.origin.orientation.y, data.info.origin.orientation.z, data.info.origin.orientation.w]
        self.mapData = data.data[:]
        #rospy.loginfo("Received Map Data: res = %d, size = (%d, %d), pos = (%0.2f, %0.2f, %0.2f) (%0.2f, %0.2f, %0.2f, %0.2f)", self.resolution, self.width, self.height, self.position[0], self.position[1], self.position[2], self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3])
        

# MapManager
#   Calculates map->odom transformation and broadcasts to tf
#   Also receives map info from map_server node and does processing on that
class MapManager():
    def __init__(self):
        self.mapData = None
        self.tfBroadcaster = tf.TransformBroadcaster()
        self.currentTf = [[0, 0, 0], [0, 0, 0, 1], rospy.Time.now()]
        self.tempTf = self.currentTf[:]
        self.tfLock = threading.Lock()  # needed in case the main thread and the filter access it at the same time
        self.rosMap = ROSMap()
        rospy.Subscriber("map", OccupancyGrid, self.rosMap.mapCallback)

    # updateMapToOdomTf(): calculate the new map->odom transformation
    # parameters:
    #   pose -- the current best guess pose (assumed to be in map coordinates)
    #   odom -- the current odometry values
    #   time -- the time at which these values were calculated
    def updateMapToOdomTf(self, pose, odom, time = None):
        self.tempTf[0][0] = pose.x - odom[0]
        self.tempTf[0][1] = pose.y - odom[1]
        self.tempTf[2] = time or rospy.Time.now()
        self.tempTf[1] = tf.transformations.quaternion_from_euler(0, 0, pose.theta-odom[2])
        self.tfLock.acquire()   # <--- grab the lock --->
        self.currentTf = self.tempTf[:]  # deep copy
        self.tfLock.release()   # <--- release the lock --->

    # broadcast(): send out the newest transform to tf
    def broadcast(self):
        self.tfLock.acquire()   # <--- grab the lock --->
        self.tfBroadcaster.sendTransform(self.currentTf[0], self.currentTf[1], self.currentTf[2], "odom", "map")
        self.tfLock.release()   # <--- release the lock --->
        #rospy.loginfo("Sent broadcast")

    # inBounds(): returns True if a given pose is in a legal position on the map
    # parameters:
    #   pose -- the pose to check against the map (assumed to be in map coordinates)
    def inBounds(self, pose):
        return True
