#!/home/mhyde/vEnvs/rosPy/bin/python

'''
node = /motorPosePub
pubTopic = /motorPoseSteps
subTopic = /arduinoState

Listens to arduino for ready state then implements
other methods to pull and convert the next coordinate
into the respective arm angles and steps.
'''

import rospy
from motor_control.msg import motorSteps
from std_msgs.msg import Bool
from utility.positionVector import positionVector
from utility.dataFilter import dataFilter

class motorPosePub(object):
    '''
    Pulls coordinates from the distributer and updates
    the current desired position for the arduino node.
    '''
    def __init__(self):
        self.positionVector = positionVector()
        self.dataFilter = dataFilter(self.positionVector)
        self.msg = motorSteps()

        self.poseSub = rospy.Subscriber('arduinoState', Bool, self.arduinoStateCallback)
        self.stepPub = rospy.Publisher('motorPoseSteps', motorSteps, queue_size=10)

        self.baseAng = 0
        self.mainAng = 0
        self.secAng = 0
        self.toolAng = 0

        rospy.spin()

    def updatePosition(self):
        '''
        Requests the next coordinate from the data
        distributing Server
        '''
        self.positionVector.pullCoor()

    def defineAngles(self):
        '''
        Uses dataFilter to return the equivalent
        angles of each motor
        '''
        self.baseAng, self.mainAng, self.secAng, \
                    self.toolAng = self.dataFilter.returnAngles()

    def defineSteps(self):
        '''
        Uses dataFilter to return the equivalent
        steps of each motor
        '''
        self.msg.baseStep, self.msg.mainStep, self.msg.secStep, \
                    self.msg.toolStep = self.dataFilter.returnSteps()

    def arduinoStateCallback(self, arduinoState):
        '''
        Updates coordinate when Arduino is ready.
        '''

        if arduinoState.data:
            self.updatePosition()
            self.defineSteps()
            self.defineAngles()
            self.publishSteps()
        else:
            pass

    def publishSteps(self):
        '''
        Publishes the steps to the motorPoseSteps topic
        for the arduino to read.
        '''
        self.stepPub.publish(self.msg)
        rospy.loginfo(self.msg)

if __name__ == '__main__':
    rospy.init_node('motorPosePub', anonymous=True)
    pub = motorPosePub()
