#!/home/mhyde/vEnvs/rosPy/bin/python

'''
ROS Service - Server:

Feeds coordinates from the CSV File to
the Data Handler upon service request.

Implements a Pandas DataFrame to contain
coordinate information as well as actuator
and motion commands.
'''

import rospy
from motor_control.srv import coordinatePass, \
                              coordinatePassResponse
from pandas import DataFrame, read_csv

class server(object):
    '''
    Coordinate Distributor

    topic =  /coordinatePass
    node  =  /coordinatePassServer

    Control via terminal using:

    rosservice call /coordinatePass 1
    '''
    def __init__(self, filePath):
        self.roboRoutine = DataFrame(read_csv(filePath, sep=","))
        self.index = 0
        self.instructionCount = 0

    def countInstructions(self):
        '''
        Counts the amount of coordinates
        in routine
        '''
        self.instructionCount = self.roboRoutine.shape[0]

    def checkIndex(self):
        '''
        Checks the current index after each service
        and sets the index back to 0 to start
        the routine over again
        '''
        if self.index < self.instructionCount - 1:
            print 'Index: %d'%self.index
            self.index += 1
        else:
            self.index = 0

    def handleRequest(self, req):
        '''
        Service Handle Function:

        Requeset Message Type - Boolean
        Any message from the client will trigger the function
        '''
        if req:
            resp = coordinatePassResponse()
            resp.xCoordinate, resp.yCoordinate, \
            resp.zCoordinate = self.roboRoutine.values[self.index, :]

            self.checkIndex()

            return resp

    def runServer(self):
        '''
        Single method callable function.
        Starts the ROS Server as long as
        it is called in an initiated node
        '''
        self.countInstructions()
        rospy.Service('coordinatePass', coordinatePass, self.handleRequest)
        rospy.spin()

if __name__ == '__main__':
    try:
        print 'Server Running ... '
        rospy.init_node('coordinatePassSever')
        server = server("~/catkin_ws/src/motor_control_Gen2/src/testCoordinates.csv")
        server.runServer()
    except rospy.ROSInterruptException():
        pass
