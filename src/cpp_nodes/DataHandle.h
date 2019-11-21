#ifndef DATAHANDLE_H_
#define DATAHANDLE_H_

class DataHandle{
private:
    float xComp, yComp, zComp;
    float PI = 3.1415;
public:
    DataHandle(float xComp, float yComp, float zComp);

    void positionVectorDef(float *length, float *pvAng);
    bool positionVectorCheck();
    float baseAngle();
    void armAngles(float *mainDegFromZ, float *secAng, float *toolAng);
    void genAngles(float *baseAng, float *mainAng, float *secAng, float *toolAng);
    
    ~DataHandle(void);
};

#endif /* DATAHANDLE_H_ */