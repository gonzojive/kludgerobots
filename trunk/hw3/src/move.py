import rospy
import viz
import select
import sys
import goal

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MoveFromKeyboard:
    def __init__(self, viz, g):
        self.velPublish = rospy.Publisher("commands", Twist) # publish to "commands"
        self.viz = viz
        self.goals = g
        self.command = None
        self.repeatCommand = 0
        self.gradient = None

    def setGradientMap(self, gmap):
        self.gradient = gmap
        
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
        #rospy.loginfo("Command received: %s", text)
        if cmd[0] == "g":   # setting or removing a goal
            if len(cmd) == 3:   # g x y
                g = goal.Goal(float(cmd[1]), float(cmd[2]))
                index = self.goals.goalExists(g)    # check if the goal already exists
                if index >= 0:  # if it does, delete it
                    rospy.loginfo("Deleted goal: (%s, %s)", cmd[1], cmd[2])
                    self.goals.deleteGoal(index)
                else:   # otherwise, add it
                    rospy.loginfo("New goal: (%s, %s)", cmd[1], cmd[2])
                    self.goals.newGoal(g)
            elif len(cmd) == 1: # just 'g' on command line means list all goals
                self.goals.logGoals()
        elif cmd[0] == "start":
            if self.gradient:
                self.gradient.setGoals(self.goals.goalList())
                if len(cmd) >= 2:
                    self.gradient.calculateCosts(int(cmd[1]))
                else:
                    self.gradient.calculateCosts()
        elif cmd[0] == "run":
            if self.gradient:
                if len(cmd) >= 2:
                    self.gradient.calculateCosts(int(cmd[1]))
                else:
                    self.gradient.calculateCosts()
        elif cmd[0] == "display":
            if cmd[1] == "intrinsic":
                if len(cmd) == 2:
                    self.gradient.displayImageOfIntrinsics()
                else:
                    self.gradient.displayImageOfIntrinsics(int(cmd[2]))
            elif cmd[1] == "cost":
                if len(cmd) == 2:
                    self.gradient.displayImageOfCosts()
                else:
                    self.gradient.displayImageOfCosts(int(cmd[2]))
            elif cmd[1] == "gradient" or cmd[1] == "grad" or cmd[1] == "g":
                self.gradient.displayGradient(self.viz)

        elif cmd[0] == "showPath":
            self.gradient.findPathGivenGradient(int(cmd[2]))
        else:
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

