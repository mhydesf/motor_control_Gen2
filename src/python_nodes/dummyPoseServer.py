#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Commuinicates with arduino to control motors
'''

import rospy
from motor_control.srv import motorPose, motorPoseResponse

def handle(data):
    '''
    Handles Incoming Request
    '''
    rospy.loginfo('[%s, %s, %s, %s]'%(data.baseAng, data.mainAng, \
                                      data.secAng, data.toolAng))
    return motorPoseResponse(True)

def server():
    '''
    Server node
    '''
    rospy.init_node('motorPoseServer')
    rospy.Service('motorPose', motorPose, handle)
    print 'Server running ... '
    rospy.spin()

if __name__ == '__main__':
    server()
