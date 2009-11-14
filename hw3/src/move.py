import rospy
import viz
import select
import sys

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MoveFromKeyboard:
    def __init__(self, viz):
        self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        self.viz = viz
        self.command = None
        self.repeatCommand = 0
        
    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self):
        text = self.keyboardInput()
        if text:
            self.parseText(text)
        if self.command:
            self.velPublish.publish(Twist(Vector3(self.command[0],0,0),Vector3(0,0,self.command[1])))
            if self.repeatCommand == 0:
                self.command = None
            if self.repeatCommand > 0:
                self.repeatCommand -= 1

    def keyboardInput(self):
        [r, w, x] = select.select([sys.stdin], [], [], 0)   # poll stdin
        if r == [sys.stdin]:
            return raw_input()

    def parseText(self, text):
        cmd = text.split()
        rospy.loginfo("Command received: %s", text)
        if len(cmd) < 2:
            rospy.loginfo("Command error: length is %d, should be at least 2", len(cmd))
            return None
        rospy.loginfo("New command: (%s, 0, 0) (0, 0, %s)", cmd[0], cmd[1])
        self.command = [float(cmd[0]), float(cmd[1])]
        if len(cmd) >= 3:
            self.repeatCommand = int(cmd[2])
        else:
            self.repeatCommand = 0

if __name__ == '__main__':
    m = MoveFromKeyboard(None)
    while 1:
        m.publishNextMovement()

