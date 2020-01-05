#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Client

Service = /coordinatePass

Object incorporated in dataPuller Client which takes
an XYZ coordiante from the data frame and coverts into
steps and angles for each of the four motors.
'''

from math import pi, atan2, acos
import rospy

class coordinateFilter(object):
    '''
    Consists of math utils for the /coordinatePass
    Service. Integrated into Service Client.
    '''
    def __init__(self, positionVector):
        self.vector = positionVector
        self.mainArmLength = rospy.get_param('mainArmLength')
        self.secArmLength = rospy.get_param('secArmLength')

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

    def returnAngles(self):
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

    def returnSteps(self):
        '''
        Takes an x, y, z coordinate and returns all motor angles.
        '''
        angles = self.returnAngles()
        steps = []
        for angle in angles:
            steps.append(int(angle / (1.8 / 4)))
        return steps

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
