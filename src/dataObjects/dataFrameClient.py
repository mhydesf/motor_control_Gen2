#!/home/mhyde/vEnvs/rosPy/bin/python

'''
ROS Service - Client:

Requests coordinates from the CSV File
to the Data Distributer upon service request.

'''

import sys
import rospy
from motor_control.srv import coordinatePass

class dataFrameClient(object):
    '''
    Actual Client for the /coordinatePass Service.
    Perpetually pulls coordinates from the dataFrame
    and feeds them to the motorPosePublisher
    '''

    def requestCoordinate(self, request):
        '''
        Function Docstring
        '''
        rospy.wait_for_service('coordinatePass')
        try:
            srvReq = rospy.ServiceProxy('coordinatePass', coordinatePass)
            resp = srvReq(request)
            return resp
        except rospy.ServiceException, error:
            rospy.loginfo('Service Call Failed: %s'%error)

    def usage(self):
        '''
        Simple Error Check in Client Request
        '''
        return '%s [req]'%sys.argv[0]

    def client(self):
        '''
        Client Implementation
        '''
        req = False
        if len(sys.argv) == 1:
            req = True
        else:
            print self.usage()
        print "Requesting Coordinate ... "
        coor = self.requestCoordinate(req)
        return coor.xCoordinate, coor.yCoordinate, coor.zCoordinate
        