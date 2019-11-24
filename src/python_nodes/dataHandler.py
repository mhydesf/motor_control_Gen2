#!/home/mhyde/vEnvs/rosPy/bin/python

'''
ROS Service - Client:

Requests coordinates from the CSV File
to the Data Distributer upon service request.

'''

import sys
import rospy
from motor_control.srv import *

def requestCoordinate(request):
    rospy.wait_for_service('coordinatePass')
    try:
        srvReq = rospy.ServiceProxy('coordinatePass', coordinatePass)
        resp = srvReq(request)
        return resp
    except rospy.ServiceException, e:
        rospy.loginfo('Service Call Failed: %s'%e)

def usage():
    return '%s [req]'%sys.argv[0]

if __name__ == '__main__':
    if len(sys.argv) == 1:
        req = True
    else:
        print usage()
    print "Requesting Coordinate ... "
    coor = requestCoordinate(req)
    print '[%s, %s, %s]'%(coor.xCoordinate, coor.yCoordinate, coor.zCoordinate)
    