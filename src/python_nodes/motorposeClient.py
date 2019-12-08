#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Commuinicates with arduino to control motors
'''

from math import pi, atan, atan2, acos, sqrt
import rospy
import serial
import dataHandler

############################################################
############################################################
############################################################

class positionVector(object):
    '''
    Properties of the vector defining the destination
    of the arm.
    '''
    def __init__(self):
        self.dataHandler = dataHandler.dataHandler()
        self.xCoor = 0
        self.yCoor = 0
        self.zCoor = 0
        self.pvAng = 0
        self.pvLength = 0
        self.pullCoor()

    def pullCoor(self):
        '''
        Pulls the coordinate from the current index
        of the dataDistributer (which is also in charge
        of tracking the current index).
        '''
        [self.xCoor, self.yCoor, self.zCoor] = self.dataHandler.client()

    def vectorCheck(self):
        '''
        Returns True if the arm can achieve the components of the position vector. This will also
        avoid any runtime errors of calculating undefined values. The domain is determined by
        the length of the arms.
        '''
        comparisonValue = sqrt(self.xCoor**2 + self.yCoor**2 + self.zCoor**2)
        return 2 <= comparisonValue <= 10

    def defineVector(self):
        '''
        Input the x, y, z coordinate of the desired poisiton and the necessary components of that
        vector will be calculated and returned. xPrime describes the horizontal length of the 2D
        vector residing in the plane already achieved by rotating the base.
        '''
        xPrime = sqrt(self.xCoor**2 + self.yCoor**2)
        self.pvAng = (atan(self.zCoor / xPrime)) * 180 / pi
        self.pvLength = sqrt(xPrime**2 + self.zCoor**2)

############################################################
############################################################
############################################################

class motorposeClient(object):
    '''
    This client is joined with the dataHandler client.
    Coordinates are pulled from the dataDistributer
    and converted into steps for each motor in the arm.
    Then the data is sent to the arduino action server
    to move the motors.
    '''
    def __init__(self):
        self.vector = positionVector()
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=3)
        self.mainArmLength = 7
        self.secArmLength = 4

    def baseAngle(self): ### I HATE THIS FUNCTION ###
        '''
        Given the x and y components of the position vector, the math.atan2() function will
        return the angle in the domain -180 <= theta <= 180. The desired domain is
        0 <= theta <= 360 so a quadrant correcting constant is passed into the lambda
        function. If the point lies in the Q1 or Q1 the quadrant corrector is 0 since the
        value returned will be between 0 and 180. If the point lies in Q3 or Q4 the quadrant
        corrector is 360 since the value returned will be between -180 and 0. It is like
        measuring forward from 0 to 180 or backward from 360 to 180. Points on quadrantals
        are also accounted for.
        '''

        angleCalc = lambda signCorrection: \
        ((atan2(self.vector.yCoor, self.vector.xCoor)) * 180 / pi) + signCorrection

        if self.vector.yCoor > 0 and self.vector.xCoor != 0:
            baseAng = angleCalc(0)
        elif self.vector.yCoor < 0 and self.vector.xCoor != 0:
            baseAng = angleCalc(360)
        elif self.vector.xCoor == 0 and self.vector.yCoor > 0:
            baseAng = 90
        elif self.vector.xCoor == 0 and self.vector.yCoor < 0:
            baseAng = 270
        elif self.vector.xCoor > 0 and self.vector.yCoor == 0:
            baseAng = 0
        elif self.vector.xCoor < 0 and self.vector.yCoor == 0:
            baseAng = 180
        return baseAng

    def armAngles(self):
        '''
        Using the law of cosignes with the length of the main arm, secondary arm,
        and position vector each angle for the arms can be calculated. The tool
        arm angle is defined by the geometry of the triangle and is set to always
        stay parallel with the ground.
        '''
        [len1, len2, len3] = [self.mainArmLength, self.secArmLength, self.vector.pvLength]
        mainDegInTri = (acos((len3**2 + len1**2 - len2**2) / (2 * len3 * len1))) * 180 / pi
        mainDegFromZ = 90 - mainDegInTri - self.vector.pvAng
        secAng = (acos((len1**2 + len2**2 - len3**2) / (2 * len1 * len2))) * 180 / pi
        angleRemainder = 180 - secAng - mainDegInTri
        toolAng = 180 - angleRemainder - self.vector.pvAng
        return mainDegFromZ, secAng, toolAng

    def genAngles(self):
        '''
        Takes an x, y, z coordinate and returns all motor angles.
        '''
        self.vector.defineVector()
        if self.vector.vectorCheck():
            baseAng = self.baseAngle()
            [mainAng, secAng, toolAng] = self.armAngles()
        else:
            coordinateErrorMsg()
            baseAng = 0
            [mainAng, secAng, toolAng] = [0, 0, 0]
        return [baseAng, mainAng, secAng, toolAng]

    def sendStepsArduino(self):
        '''
        Sends each motors steps to arduino upon
        request. Waits for messsage from arduino
        to grab, convert, and send the next
        coordinate.
        '''
        self.ser.write('2000')
        print 'Wrote Data'

    def closePort(self):
        '''
        Closes serial port
        '''
        self.ser.close()

############################################################
############################################################
############################################################

def coordinateErrorMsg():
    '''
    Error Message when a coordinate is outside the bounds of the arms capabilities.
    '''
    print '''


=========================================
        COORDINATE NOT VALID!!!
    PLEASE CORRECT THE INSTRUCTION...

    ARM RETURNING TO HOME POSITION...
=========================================


    '''

############################################################
############################################################
############################################################

if __name__ == '__main__':
    rospy.init_node('motorPoseClient')
    mpClient = motorposeClient()

    while True:
        userInput = raw_input("Do you want to send data ... [Y/n]:  ")
        if userInput.upper() == 'Y':
            mpClient.sendStepsArduino()
        if userInput == 'end':
            break
