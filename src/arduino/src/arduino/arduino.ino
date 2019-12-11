#define resolutionL1        2     //Base And Main Step Resolution Pin1
#define resolutionL2        3     //Base And Main Step Resolution Pin2
#define resolutionR1        8     //Sec. And Tool Step Resolution Pin1
#define resolutionR2        9     //Sec. And Tool Step Resolution Pin2

#define baseStepPin         4
#define baseDirPin          5

#define mainStepPin         6
#define mainDirPin          7

#define secStepPin          10
#define secDirPin           11

#define toolStepPin         12
#define toolDirPin          13

#define baseStepH           PORTD |=  0b00010000;     //Writing PORTB Register (Pin 4) High
#define baseStepL           PORTD &= ~0b00010000;     //Writing PORTB Register (Pin 4) Low

#define mainStepH           PORTD |=  0b01000000;     //Writing PORTB Register (Pin 6) High
#define mainStepL           PORTD &= ~0b01000000;     //Writing PORTB Register (Pin 6) Low

#define secStepH            PORTB |=  0b00000100;     //Writing PORTB Register (Pin 10) High
#define secStepL            PORTB &= ~0b00000100;     //Writing PORTB Register (Pin 10) Low

#define toolStepH           PORTB |=  0b00010000;     //Writing PORTB Register (Pin 12) High
#define toolStepL           PORTB &= ~0b00010000;     //Writing PORTB Register (Pin 12) Low

#define NumSteppers         4

#include "ros.h"
#include "motor_control/motorSteps.h"
#include "std_msgs/Empty.h"

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

struct StepperDef{
  void (*dirFunc)(int);
  void (*stepFunc)();

  long stepPosition;
  long stepGoal;
};

volatile StepperDef steppers[NumSteppers];

void setup(){

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

    digitalWrite(resolutionL1, LOW);
    digitalWrite(resolutionL2, HIGH);
    digitalWrite(resolutionR1, LOW);
    digitalWrite(resolutionR2, HIGH);

    steppers[0].dirFunc = baseDir;
    steppers[0].stepFunc = baseStep;

    steppers[1].dirFunc = mainDir;
    steppers[1].stepFunc = mainStep;

    steppers[2].dirFunc = secDir;
    steppers[2].stepFunc = secStep;

    steppers[3].dirFunc = toolDir;
    steppers[3].stepFunc = toolStep;
}

void loop(){
    
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
