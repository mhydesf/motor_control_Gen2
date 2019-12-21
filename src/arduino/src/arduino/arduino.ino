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

//////////////////////////////////////////////////////////////////////////////////

///////////////                 Global Parameters                  ///////////////

//////////////////////////////////////////////////////////////////////////////////

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

struct StepperDef{              /****Stepper Structure****/
    void (*dirFunc)(int);       //Discrete Direction Function (Takes 0/1 ::: Low/High)
    void (*stepFunc)();         //Discrete Step Function

    int stepInc;                //Step Increment (+1 for CW Motion ::: -1 for CCW Motion)

    long previousPosition;      //Position in Steps before more started
    long currentPosition;       //Position in Steps per discrete motion
    long goalPosition;          //Position in Steps that needs to be achieved
    long discreteStep;          //Steps for goal to goal
};

volatile StepperDef steppers[NumSteppers];  //List of Steppers

#include <ros.h>
#include <motor_control/motorSteps.h>

ros::NodeHandle  node;

void messageCb(motor_control::motorSteps &msg){     /* Defines Goal For Each MotorPose Step Publication */
    steppers[0].goalPosition = msg.baseStep;
    steppers[1].goalPosition = msg.mainStep;
    steppers[2].goalPosition = msg.secStep;
    steppers[3].goalPosition = msg.toolStep;
}

ros::Subscriber<motor_control::motorSteps> poseSub("motorPoseSteps", &messageCb );

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
    steppers[0].goalPosition        = 0;

    steppers[1].dirFunc             = mainDir;
    steppers[1].stepFunc            = mainStep;
    steppers[1].currentPosition     = 0;
    steppers[1].previousPosition    = 0;
    steppers[1].goalPosition        = 0;

    steppers[2].dirFunc             = secDir;
    steppers[2].stepFunc            = secStep;
    steppers[2].currentPosition     = 0;
    steppers[2].previousPosition    = 0;
    steppers[2].goalPosition        = 0;

    steppers[3].dirFunc             = toolDir;
    steppers[3].stepFunc            = toolStep;
    steppers[3].currentPosition     = 0;
    steppers[3].previousPosition    = 0;
    steppers[3].goalPosition        = 0;

    node.initNode();
    node.subscribe(poseSub);
}

//Arduino Main Loop
void loop(){
    positionMotors();
    node.spinOnce();
}

void baseStep(){                        //Base Step Function
    baseStepH
    baseStepL
}

void baseDir(int dir){                  //Base Direction Function
    digitalWrite(baseDirPin, dir);
}

void mainStep(){                        //Main Step Function
    mainStepH
    mainStepL
}

void mainDir(int dir){                  //Main Direction Function
    digitalWrite(mainDirPin, dir);
}

void secStep(){                         //Sec Step Function
    secStepH
    secStepL
}

void secDir(int dir){                   //Sec Direction Function
    digitalWrite(secDirPin, dir);
}

void toolStep(){                        //Tool Step Function
    toolStepH
    toolStepL
}

void toolDir(int dir){                  //Tool Direction Function
    digitalWrite(toolDirPin, dir);
}

void setDiscreteSteps(){
    //Sets the direction and amount of steps per positioning cycle
    for (int i = 0; i < NumSteppers; i++){
        steppers[i].discreteStep = steppers[i].goalPosition - steppers[i].currentPosition;
        if (steppers[i].discreteStep >= 0){     // IF Discrete Steps Positive
            steppers[i].dirFunc(0);             // Move CCW
            steppers[i].stepInc = 1;            // Define Motion as (+)
        }
        else if (steppers[i].discreteStep < 0){
            steppers[i].discreteStep = abs(steppers[i].discreteStep);   // Convert CW Motion to Positive Steps
            steppers[i].dirFunc(1);                                     // Move CW
            steppers[i].stepInc = -1;                                   // Define Motion as (-)
        }
    }
}

int maxSteps(){
    //Returns the maximum steps from the largest goal step position
    return max(max(max(steppers[0].discreteStep, steppers[1].discreteStep), steppers[2].discreteStep), steppers[3].discreteStep);
}

void positionMotors() {
    setDiscreteSteps();                                                         // Define Required Steps to Goal
    int currMaxSteps = maxSteps();                                              // Return Largest Step Difference
    for (int i = 0; i < currMaxSteps; i++){                     
        for (int j = 0; j < NumSteppers; j++){
            if (i < steppers[j].discreteStep){                                  // IF there are steps to take
                steppers[j].stepFunc();                                         // Step
                steppers[j].currentPosition += steppers[j].stepInc;             // Update Current Position
            }
            else{                                                               // Motion is Done
                steppers[j].previousPosition = steppers[j].currentPosition;     // Update Previous Position`
                steppers[j].goalPosition = steppers[j].currentPosition;         // Update Goal to hold position until new position is defined
            }
            delay(1);                                                           // Defines speed per interval
        }
    }
}
