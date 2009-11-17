import rospy
import viz
import select
import sys
import goal
from vector import *
import util
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
        self.localGradient = None

    def setGradientMaps(self, gmap, local):
        self.gradient = gmap
        self.localGradient = local
        
    # this is called by the main update loop of the program.  It uses the robot global compass
    # and laser interpreter to figure out where the obstacles are and move around
    def publishNextMovement(self):
        text = self.keyboardInput()
        if text:
            try:
                self.parseText(text)
            except:
                rospy.loginfo("Error with syntax!!!!")
                raise
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
            
    def getNextCommand(self,currPose):   
        rospy.loginfo("currPose is:(%f,%f,%f)",currPose.x,currPose.y,currPose.theta)
        nearestGridPoint = self.gradient.cellNearestXY(currPose.x, currPose.y) #returns a gradientMap value
        newGrad = self.gradient.interpolateGradientAtXY(currPose.x,currPose.y)
        #newPose = vector_add([currPose.x,currPose.y], vector_scale(interpedGrad, self.gradient.stepSize))
        newTheta = math.atan2(newGrad[1],newGrad[0])
        rospy.loginfo("going to (%f)",newTheta)
        goals = [[g.x, g.y] for g in self.gradient.goals]
        
        #keep turning without translation until we are in line with the destination
        if abs(currPose.theta - newTheta) > 1:
            if currPose.theta > newTheta:
                angVel = -0.2
            else:
                angVel = 0.2
            linVel = 0.01
        elif abs(currPose.theta - newTheta) > 0.05:
            if currPose.theta > newTheta:
                angVel = -0.04
            else:
                angVel = 0.04
            linVel = 0.01
        else:
            angVel = 0
            linVel = 0.05
        if util.closeToOne([currPose.x,currPose.y], goals,0.5):
            linVel = 0.03
        if util.closeToOne([currPose.x,currPose.y], goals,0.3):
            linVel = 0
            angVel = 0
            rospy.loginfo("reached goal");
        rospy.loginfo("sending linearVel = %f,angVel =%f",linVel,angVel)
        return [linVel,angVel]
            
        
        
     

    def parseText(self, text):
        cmd = text.split()
        rospy.loginfo("received command: %s", text)
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
            self.gradient.findPathGivenGradient(self.viz)
        elif cmd[0] == "test":
            if len(cmd) == 1 or cmd[1] == "1":
                rospy.loginfo("Setting goal (40, 45)")
                g = goal.Goal(40, 45)
                self.goals.newGoal(g)
                self.gradient.setGoals(self.goals.goalList())
                rospy.loginfo("Iterating 50 costs")
                self.gradient.calculateCosts(50)
                rospy.loginfo("Displaying gradient field")
                self.gradient.displayGradient(self.viz)
                rospy.loginfo("Calculating optimal path")
                self.gradient.findPathGivenGradient(self.viz)
                rospy.loginfo("Done with test")
        elif cmd[0] == "moveRobot":
            rospy.loginfo("Setting goal (40, 45)")
            g = goal.Goal(40, 45)
            self.goals.newGoal(g)
            self.gradient.setGoals(self.goals.goalList())
            rospy.loginfo("Iterating 50 costs")
            self.gradient.calculateCosts(50)
            rospy.loginfo("Displaying gradient field")
            self.gradient.displayGradient(self.viz)
        elif cmd[0] == "move" or cmd[0] == "m":
            rospy.loginfo("New command: (%s, 0, 0) (0, 0, %s)", cmd[1], cmd[2])
            self.command = [float(cmd[1]), float(cmd[2])]
            if len(cmd) >= 4:
                self.repeatCommand = int(cmd[3])
            else:
                self.repeatCommand = 0
        else:
            rospy.loginfo("Not a valid command")
            

if __name__ == '__main__':
    m = MoveFromKeyboard(None)
    while 1:
        m.publishNextMovement()
        


