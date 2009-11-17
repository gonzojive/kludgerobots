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
        #rospy.loginfo("currPose is:(%f,%f,%f)",currPose.x,currPose.y,currPose.theta)
        nearestGridPoint = self.gradient.cellNearestXY(currPose.x, currPose.y) #returns a gradientMap value
        newGrad = self.gradient.interpolateGradientAtXY(currPose.x,currPose.y)
        #newPose = vector_add([currPose.x,currPose.y], vector_scale(interpedGrad, self.gradient.stepSize))
        newTheta = math.atan2(newGrad[1],newGrad[0])
        #rospy.loginfo("going to (%f)",newTheta)
        goals = [[g.x, g.y] for g in self.goals.goalList()]
        if len(goals) == 0:
            return [0, 0]
        poseToGrad = util.normalizeAngle360(currPose.theta - newTheta)
        gradToPose = util.normalizeAngle360(newTheta - currPose.theta)
        thetaDiff = min(poseToGrad, gradToPose)
        if thetaDiff > math.pi*0.5 and thetaDiff < math.pi*1.5:
            if poseToGrad < math.pi:
                angVel = -0.2
            else:
                angVel = 0.2
            linVel = 0.0
        elif thetaDiff > math.pi/4.0 or thetaDiff < math.pi*7.0/4.0:
            if poseToGrad < math.pi:
                angVel = -0.1
            else:
                angVel = 0.1
            linVel = 0.15
        elif thetaDiff > math.pi/8.0 or thetaDiff < math.pi*15.0/8.0:
            if poseToGrad < math.pi:
                angVel = -0.05
            else:
                angVel = 0.05
            linVel = 0.2
        else:
            angVel = 0.0
            linVel = 0.3
        if util.closeToOne([currPose.x,currPose.y], goals, 1.0):
            linVel = linVel / 2.0
        if util.closeToOne([currPose.x,currPose.y], goals, 0.2):
            i = 0
            # find the index of the goal we're on
            for i in range(len(goals)):
                if util.close([currPose.x, currPose.y], goals[i], 0.2):
                    break
            linVel = 0
            angVel = 0
            rospy.loginfo("Reached goal at (%0.2f, %0.2f)", self.goals.goalList()[i].x, self.goals.goalList()[i].y)
            self.goals.deleteGoal(i)
            self.gradient.setStartPosition(currPose.x, currPose.y)
            self.gradient.setGoals(self.goals.goalList())
            self.gradient.calculateCosts(70)
            self.gradient.displayGradient(self.viz)
        #rospy.loginfo("sending linearVel = %f,angVel =%f",linVel,angVel)
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
            elif cmd[1] == "cost2":
                if len(cmd) == 2:
                    self.gradient.displayImageOfCosts2()
                else:
                    self.gradient.displayImageOfCosts(int(cmd[2]))
            elif cmd[1] == "gradient" or cmd[1] == "grad" or cmd[1] == "g":
                self.gradient.displayGradient(self.viz)
            elif cmd[1] == "local":
                if cmd[2] == "intrinsic":
                    if len(cmd) == 3:
                        self.localGradient.displayImageOfIntrinsics()
                    else:
                        self.localGradient.displayImageOfIntrinsics(int(cmd[3]))
                elif cmd[2] == "cost":
                    if len(cmd) == 3:
                        self.localGradient.displayImageOfCosts()
                    else:
                        self.localGradient.displayImageOfCosts(int(cmd[3]))
                elif cmd[2] == "gradient" or cmd[2] == "grad" or cmd[2] == "g":
                    self.localGradient.displayGradient(self.viz)
        elif cmd[0] == "showPath":
            self.gradient.findPathGivenGradient(self.viz)
            if len(cmd) > 1:
                if cmd[1] == "local":
                    lf.localGradient.findPathGivenGradient(self.viz)
        elif cmd[0] == "sanity":
            self.gradient.sanityCheck()
        elif cmd[0] == "test":
            if len(cmd) == 1 or cmd[1] == "1":
                rospy.loginfo("Setting goal (40, 45)")
                g = goal.Goal(40, 45)
                self.goals.newGoal(g)
                self.gradient.setGoals(self.goals.goalList())
                rospy.loginfo("Iterating 60 costs")
                self.gradient.calculateCosts(60)
                rospy.loginfo("Displaying gradient field")
                self.gradient.displayGradient(self.viz)
                rospy.loginfo("Calculating optimal path")
                self.gradient.findPathGivenGradient(self.viz)
                rospy.loginfo("Done with test")
        elif cmd[0] == "moveRobot":
            rospy.loginfo("Setting goal (40, 45)")
            g = goal.Goal(40, 45)
            self.goals.newGoal(g)
            g = goal.Goal(36, 47)
            self.goals.newGoal(g)
            g = goal.Goal(40, 43)
            self.goals.newGoal(g)
            self.gradient.setGoals(self.goals.goalList())
            rospy.loginfo("Iterating 60 costs")
            self.gradient.calculateCosts(60)
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
        


