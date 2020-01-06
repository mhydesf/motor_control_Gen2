#!/home/mhyde/vEnvs/rosPy/bin/python

'''
CLASS DOC
'''

import rospy

class stepper(object):
    '''
    CLASS DOC
    '''
    def __init__(self, params):

        self.name = params[0]

        self.directionPin = rospy.get_param(params[1])
        self.stepPin = rospy.get_param(params[2])

        self.currentPosition = rospy.get_param(params[3])
        self.goalPosition = rospy.get_param(params[3])

        self.gearRatio = rospy.get_param(params[4])

    def updateGoal(self, goal):
        self.goalPosition = goal
        rospy.loginfo('%s: Step Position - %s'%(self.name, goal))

    def setDirection(self):
        if self.goalPosition - self.currentPosition > 0:
            return 0
        else:
            return 1
    
    def stepsToComplete(self):
        return abs(self.goalPosition - self.currentPosition)
