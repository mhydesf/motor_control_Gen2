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

        self.directionPin = rospy.get_param(params[0])
        self.stepPin = rospy.get_param(params[1])

        self.currentPosition = rospy.get_param(params[2])
        self.goalPosition = rospy.get_param(params[2])

        self.gearRatio = rospy.get_param(params[3])

    def updateGoal(self):
        pass

    def setDirection(self):
        pass
    
    def stepsToComplete(self):
        pass

if __name__ == '__main__':
    baseStepDef = ['baseDirPin', 'baseStepPin', 'baseHome', 'baseGR']
    mainStepDef = ['mainDirPin', 'mainStepPin', 'mainHome', 'mainGR']
    secStepDef = ['secDirPin', 'secStepPin', 'secHome', 'secGR']
    toolStepDef = ['toolDirPin', 'toolStepPin', 'toolHome', 'toolGR']
    baseStepper = stepper(baseStepDef)
    mainStepper = stepper(mainStepDef)
    secStepper = stepper(secStepDef)
    toolStepper = stepper(toolStepDef)
