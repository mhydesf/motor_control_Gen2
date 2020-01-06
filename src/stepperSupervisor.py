#!/home/mhyde/vEnvs/rosPy/bin/python

'''
node = /motorPosePub
pubTopic = /motorPoseSteps
subTopic = /arduinoState

Listens to arduino for ready state then implements
other methods to pull and convert the next coordinate
into the respective arm angles and steps.
'''

from hardwareObjects.stepper import stepper
from utility.positionVector import positionVector
from utility.coordinateFilter import coordinateFilter

baseStepDef = ['Base Motor', 'baseDirPin', 'baseStepPin', 'baseHome', 'baseGR']
mainStepDef = ['Main Arm Motor', 'mainDirPin', 'mainStepPin', 'mainHome', 'mainGR']
secStepDef = ['Second Arm Motor', 'secDirPin', 'secStepPin', 'secHome', 'secGR']
toolStepDef = ['Tool Fixture Motor', 'toolDirPin', 'toolStepPin', 'toolHome', 'toolGR']

class stepperImp(object):
    '''
    Pulls coordinates from the distributer and updates
    the current desired position for the arduino node.
    '''
    def __init__(self):

        self.positionVector = positionVector()
        self.coordinateFilter = coordinateFilter(self.positionVector)

        self.baseStepper = stepper(baseStepDef)
        self.mainStepper = stepper(mainStepDef)
        self.secStepper = stepper(secStepDef)
        self.toolStepper = stepper(toolStepDef)

    def updateGoalCoordinate(self):
        '''
        Requests the next coordinate from the data
        distributing Server
        '''
        self.positionVector.pullCoor()

    def defineSteps(self):
        '''
        Uses coordinateFilter to return the equivalent
        steps of each motor
        '''
        self.baseStepper.goalPosition, self.mainStepper.goalPosition, self.secStepper.goalPosition, \
                    self.toolStepper.goalPosition = self.coordinateFilter.returnSteps()
