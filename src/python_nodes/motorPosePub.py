#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Publisher Method for Data Distribution
'''

import rospy
from motor_control.msg import motorSteps
from positionVector import positionVector

class motorPosePub(object):
    '''
    Pulls coordinates from the distributer and updates
    the current desired position for the arduino node.
    '''
    def __init__(self):
        self.positionVector = positionVector()
        self.msg = motorSteps()
        self.pub = rospy.Publisher('motorSteps', motorSteps, queue_size=10)

    def updatePosition(self):
        '''
        Requests the next coordinate from the data
        distributing Server
        '''
        self.positionVector.pullCoor()

if __name__ == '__main__':
    rospy.init_node('motorPosePub', anonymous=True)
    pub = motorPosePub()
    for _ in range(5):
        pub.updatePosition()
        print "[%d, %d, %d]"%(pub.positionVector.xCoor, \
                pub.positionVector.yCoor, pub.positionVector.zCoor)
