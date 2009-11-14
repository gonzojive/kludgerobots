import rospy
import roslib

# Goal class
# contains x and y values
class Goal:
    def __init__(self, goalX, goalY):
        self.x = goalX
        self.y = goalY
    def toStr(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"

# GoalSet class
# contains a list of all the goals, functions for adding and deleting
class GoalSet:
    # __init__()
    # no arguments
    def __init__(self):
        self.goals = []

    # newGoal()
    # takes in a new Goal object and appends it to the list
    def newGoal(self, g):
        self.goals.append(g)

    # deleteGoal()
    # takes in an index, deletes that goal from the list
    def deleteGoal(self, index):
        del self.goals[index]

    # goalExists()
    # takes in a Goal objects, returns the index if it exists, -1 otherwise
    def goalExists(self, goal):
        i = 0
        for g in self.goals:
            if g.x == goal.x and g.y == goal.y:
                return i
            i += 1
        return -1

    # logGoals()
    # prints the current goals to rxconsole
    def logGoals(self):
        if( len(self.goals) > 0 ):
            rospy.loginfo("Current goals:")
            for g in self.goals:
                rospy.loginfo("  " + g.toStr())
        else:
            rospy.loginfo("No goals have been set")
    
