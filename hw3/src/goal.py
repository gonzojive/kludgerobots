import rospy
import roslib

class Goal:
    def __init__(self, goalX, goalY):
        self.x = goalX
        self.y = goalY
    def toStr(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"

class GoalSet:
    def __init__(self):
        self.goals = []
    def newGoal(self, g):
        self.goals.append(g)
    def deleteGoal(self, index):
        del self.goals[index]
    def goalExists(self, goal):
        i = 0
        for g in self.goals:
            if g.x == goal.x and g.y == goal.y:
                return i
            i += 1
        return -1
    def logGoals(self):
        if( len(self.goals) > 0 ):
            rospy.loginfo("Current goals:")
            for g in self.goals:
                rospy.loginfo("  " + g.toStr())
        else:
            rospy.loginfo("No goals have been set")
    
