#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Commuinicates with arduino to control motors
'''
from math import atan, pi, sqrt
from dataPuller import dataPuller

class positionVector(object):
    '''
    Properties of the vector defining the destination
    of the arm.
    '''
    def __init__(self):
        self.dataPuller = dataPuller()
        self.xCoor = 0
        self.yCoor = 0
        self.zCoor = 0
        self.pvAng = 0
        self.pvLength = 0

    def pullCoor(self):
        '''
        Pulls the coordinate from the current index
        of the dataDistributer (which is also in charge
        of tracking the current index).
        '''
        [self.xCoor, self.yCoor, self.zCoor] = self.dataPuller.client()

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
