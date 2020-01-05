#include <Arduino.h>

void setup();
void loop();
void baseStep();
void baseDir(int dir);
void mainStep();
void mainDir(int dir);
void secStep();
void secDir(int dir);
void toolStep();
void toolDir(int dir);
void setDirection();
void move();
#line 1 "src/arduino.ino"
/*
ROSserial_Arduino Node for the 4 DOF Robotic Arm Motor_Control
ROS Package. Hosts a publisher to /arduinoState and a subscriber
to /motorPoseSteps.

The Arduino will publish a Boolean to the /arduinoState topic to
let the system node it is ready for an update coordinate.
The Subscriber will update the relevant positions in steps and
the motors will adjust themselves until that position is achieved.
At this point, the arduino will let the system know it is ready for
the next position.
*/

#define resolutionL1        2     //Base And Main Step Resolution Pin1
#define resolutionL2        3     //Base And Main Step Resolution Pin2
#define resolutionR1        8     //Sec. And Tool Step Resolution Pin1
#define resolutionR2        9     //Sec. And Tool Step Resolution Pin2

#define baseStepPin         4     //Base Step Pin
#define baseDirPin          5     //Base Dir Pin

#define mainStepPin         6     //Main Step Pin
#define mainDirPin          7     //Main Dir Pin

#define secStepPin          10    //Sec Step Pin
#define secDirPin           11    //Sec Dir Pin

#define toolStepPin         12    //Tool Step Pin
#define toolDirPin          13    //Tool Dir Pin

#define baseStepH           PORTD |=  0b00010000;     //Writing PORTB Register (Pin 4) High
#define baseStepL           PORTD &= ~0b00010000;     //Writing PORTB Register (Pin 4) Low

#define mainStepH           PORTD |=  0b01000000;     //Writing PORTB Register (Pin 6) High
#define mainStepL           PORTD &= ~0b01000000;     //Writing PORTB Register (Pin 6) Low

#define secStepH            PORTB |=  0b00000100;     //Writing PORTB Register (Pin 10) High
#define secStepL            PORTB &= ~0b00000100;     //Writing PORTB Register (Pin 10) Low

#define toolStepH           PORTB |=  0b00010000;     //Writing PORTB Register (Pin 12) High
#define toolStepL           PORTB &= ~0b00010000;     //Writing PORTB Register (Pin 12) Low

#define NumSteppers         4     //Number of Steppers in System

//Stepper Structure
struct StepperDef{
    void (*dirFunc)(int);       //Discrete Direction Function (Takes 0/1 ::: High/Low)
    void (*stepFunc)();         //Discrete Step Function

    int stepInc = 1;            //Counts step (+/-) 1

    long previousPosition;      //Position in Steps before more started
    long currentPosition;       //Position in Steps per discrete motion
    long goalPosition;          //Position in Steps that needs to be achieved
};

//List of Steppers defined as a list of length 4
volatile StepperDef steppers[NumSteppers];

//Arduino Setup Function. Run Once upon upload and reset
void setup(){

    //Setting Pin Modes
    pinMode(resolutionL1,      OUTPUT);
    pinMode(resolutionL2,      OUTPUT);
    pinMode(resolutionR1,      OUTPUT);
    pinMode(resolutionR2,      OUTPUT);
    pinMode(baseStepPin,       OUTPUT);
    pinMode(baseDirPin,        OUTPUT);
    pinMode(mainStepPin,       OUTPUT);
    pinMode(mainDirPin,        OUTPUT);
    pinMode(secStepPin,        OUTPUT);
    pinMode(secDirPin,         OUTPUT);
    pinMode(toolStepPin,       OUTPUT);
    pinMode(toolDirPin,        OUTPUT);

    //Set Default Step Resolution
    digitalWrite(resolutionL1, LOW);
    digitalWrite(resolutionL2, HIGH);
    digitalWrite(resolutionR1, LOW);
    digitalWrite(resolutionR2, HIGH);

    //Assigning Step and Dir functions to Stepper List
    steppers[0].dirFunc             = baseDir;
    steppers[0].stepFunc            = baseStep;
    steppers[0].currentPosition     = 0;
    steppers[0].previousPosition    = 0;
    steppers[0].goalPosition        = 200;

    steppers[1].dirFunc             = mainDir;
    steppers[1].stepFunc            = mainStep;
    steppers[1].currentPosition     = 0;
    steppers[1].previousPosition    = 0;
    steppers[1].goalPosition        = 400;

    steppers[2].dirFunc             = secDir;
    steppers[2].stepFunc            = secStep;
    steppers[2].currentPosition     = 0;
    steppers[2].previousPosition    = 0;
    steppers[2].goalPosition        = 800;

    steppers[3].dirFunc             = toolDir;
    steppers[3].stepFunc            = toolStep;
    steppers[3].currentPosition     = 0;
    steppers[3].previousPosition    = 0;
    steppers[3].goalPosition        = 1600;

}

//Arduino Main Loop
void loop(){
    move();
}

void baseStep(){
    baseStepH
    baseStepL
}

void baseDir(int dir){
    digitalWrite(baseDirPin, dir);
}

void mainStep(){
    mainStepH
    mainStepL
}

void mainDir(int dir){
    digitalWrite(mainDirPin, dir);
}

void secStep(){
    secStepH
    secStepL
}

void secDir(int dir){
    digitalWrite(secDirPin, dir);
}

void toolStep(){
    toolStepH
    toolStepL
}

void toolDir(int dir){
    digitalWrite(toolDirPin, dir);
}

void setDirection(){
    for (int i = 0; i < 4; i++){
        if (steppers[i].goalPosition < steppers[i].currentPosition){
            steppers[i].dirFunc(0);
            //steppers[i].stepInc = 1;
        }
        else{
            steppers[i].dirFunc(1);
            //steppers[i].stepInc = -1;
        }
    }
}

void move(){
    setDirection();
    for (int i = 0; i < 4; i++){
        if (steppers[i].currentPosition < steppers[i].goalPosition){
            steppers[i].stepFunc();
            steppers[i].currentPosition += steppers[i].stepInc;
        }
    }
    delay(10);
}