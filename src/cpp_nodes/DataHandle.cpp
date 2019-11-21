#include <iostream>
#include <math.h>

#include "DataHandle.h"

using namespace std;

DataHandle::DataHandle(float xComp, float yComp, float zComp){
    this->xComp = xComp;
    this->yComp = yComp;
    this->zComp = zComp;
}

void DataHandle::positionVectorDef(float *length, float *pvAng){
    float xPrime = sqrt(pow(xComp, 2) + pow(yComp, 2));
    *pvAng = (atan(zComp / xPrime)) * 180 / PI;
    *length = sqrt(pow(xPrime, 2) + pow(zComp, 2));
}

bool DataHandle::positionVectorCheck(){
    float comparisonValue = sqrt(pow(xComp, 2) + pow(yComp, 2) + pow(zComp, 2));
    return 2 <= comparisonValue <= 10;
    }

float DataHandle::baseAngle(){
    float baseAng;
    if (yComp > 0){
        baseAng = (atan2(yComp, xComp)) * 180 / PI;
    }
    else if (yComp < 0){
        baseAng = ((atan2(yComp, xComp)) * 180 / PI) + 360;
    }
    else if (xComp == 0 && yComp > 0){
        baseAng = 90;
    }
    else if (xComp == 0 && yComp < 0){
        baseAng = 270;
    }
    else if (xComp > 0 && yComp == 0){
        baseAng = 0;
    }
    else if (xComp < 0 && yComp == 0){
        baseAng = 180;
    }

    return baseAng;
}

void DataHandle::armAngles(float *mainDegFromZ, float *secAng, float *toolAng){

}

void DataHandle::genAngles(float *baseAng, float *mainAng, float *secAng, float *toolAng){

}

DataHandle::~DataHandle(void){
    cout << "Destroying Class ... " << endl;
}