#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Publisher Method for Data Distribution
'''

import rospy
from motor_control.msg import motorSteps
from positionVector import positionVector
from dataConverter import dataConverter

class motorPosePub(object):
    '''
    Pulls coordinates from the distributer and updates
    the current desired position for the arduino node.
    '''
    def __init__(self):
        self.positionVector = positionVector()
        self.dataConverter = dataConverter(self.positionVector)
        self.msg = motorSteps()
        self.pub = rospy.Publisher('motorPoseSteps', motorSteps, queue_size=10)

        self.baseAng = 0
        self.mainAng = 0
        self.secAng = 0
        self.toolAng = 0

    def updatePosition(self):
        '''
        Requests the next coordinate from the data
        distributing Server
        '''
        self.positionVector.pullCoor()

    def defineAngles(self):
        '''
        Uses dataConverter to return the equivalent
        angles of each motor
        '''
        self.baseAng, self.mainAng, self.secAng, \
                    self.toolAng = self.dataConverter.returnAngles()

    def defineSteps(self):
        '''
        Uses dataConverter to return the equivalent
        steps of each motor
        '''
        self.msg.baseStep, self.msg.mainStep, self.msg.secStep, \
                    self.msg.toolStep = self.dataConverter.returnSteps()

    def publishSteps(self):
        '''
        Publishes the steps to the motorPoseSteps topic
        for the arduino to read.
        '''
        rate = rospy.Rate(2)
        self.updatePosition()
        self.defineSteps()
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            rospy.loginfo(self.msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('motorPosePub', anonymous=True)
    pub = motorPosePub()
    pub.publishSteps()
