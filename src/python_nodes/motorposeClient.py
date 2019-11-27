#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Commuinicates with arduino to control motors
'''

import rospy
from motor_control.srv import motorPose, motorPoseRequest
import dataHandler

def sendSteps(baseSteps, mainSteps, secSteps):
    '''
    Collect Data from Data Handler and
    Send to motorposeServer to control
    arm position
    '''
    rospy.wait_for_service('motorPose')

    client = rospy.ServiceProxy('motorPose', motorPose)
    motorSteps = motorPoseRequest()
    motorSteps.baseAng = baseSteps
    motorSteps.mainAng = mainSteps
    motorSteps.secAng = secSteps
    #motorSteps.toolAng = toolSteps

    print client(motorSteps)

if __name__ == '__main__':
    try:
        data = dataHandler.dataHandler()
        steps = data.client()
        rospy.init_node('motorposeClient', anonymous=True)
        sendSteps(steps[0], steps[1], steps[2])
    except rospy.ROSInterruptException:
        pass
